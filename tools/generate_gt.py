#!/usr/bin/env python3
"""
离线真值轨迹生成器 (纯 Python, 零 rosbags/rosbag2_py 依赖)

三种配准方法:
  gicp:      Open3D GICP 帧到地图独立配准
  pgo:       因子图优化 (帧间GICP + 地图先验 + 锚点)
  odom_gicp: 腿部里程计帧间预测 + 帧到地图GICP修正 (默认, 推荐)

模式:
  模式A [--bag]:     sqlite3 直读 .db3 → 手动 CDR 解析 → 配准 → TUM 轨迹
  模式B [--pcd_dir]: 已有 PCD 帧序列 → 配准 → TUM 轨迹

用法:
  python3 generate_gt.py --bag ~/下载/rosbag/5#GJD --map map.pcd --output gt.txt
  python3 generate_gt.py --bag ~/下载/rosbag/5#GJD --map map.pcd --method gicp --output gt.txt
  python3 generate_gt.py --pcd_dir /tmp/pcd_frames --map map.pcd --output gt.txt
"""

import argparse, sqlite3, struct, sys
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import open3d as o3d

# ─── 外参 ───
T_L1_IMU = np.eye(4); T_L1_IMU[:3,3] = [0.34218, 0.0, 0.17851]
T_L2_IMU = np.eye(4)
T_L2_IMU[:3,:3] = [[0.866025, 0.0, 0.5],[0.0, 1.0, 0.0],[-0.5, 0.0, 0.866025]]
T_L2_IMU[:3,3] = [-0.3, 0.0, 0.15]
GICP_ITER=500; GICP_EPS=1e-6; GICP_CORR=3.0; VOXEL_MAP=0.5; VOXEL_SCAN=0.3

# ─── 手动 CDR 解析 ───
def r_u32(d,o): return struct.unpack_from('<I',d,o)[0], o+4
def r_i32(d,o): return struct.unpack_from('<i',d,o)[0], o+4
def r_u8(d,o):  return d[o], o+1
def r_f64(d,o): return struct.unpack_from('<d',d,o)[0], o+8

def r_string(d, o):
    L, o = r_u32(d, o)
    s = d[o:o+L].split(b'\x00')[0].decode('utf-8','replace')
    return s, o + ((L+3)&~3)

def parse_pc2_cdr(data: bytes):
    """手动解析 sensor_msgs/PointCloud2 CDR → (stamp, xyz_pts)."""
    off = 0
    off += 4  # header.seq
    sec, off = r_i32(data, off); nsec, off = r_u32(data, off)
    stamp = sec + nsec*1e-9
    _, off = r_string(data, off)  # frame_id
    height, off = r_u32(data, off); width, off = r_u32(data, off)
    nf, off = r_u32(data, off)  # fields sequence length
    fields = []
    for _ in range(nf):
        name, off = r_string(data, off)
        foff, off = r_u32(data, off)
        dtype = data[off]; off += 1
        off = (off + 3) & ~3  # CDR align after uint8
        count, off = r_u32(data, off)
        fields.append((name, foff, dtype, count))
    is_be = data[off]; off += 1
    off = (off + 3) & ~3  # CDR align after uint8
    pt_step, off = r_u32(data, off); row_step, off = r_u32(data, off)
    data_len, off = r_u32(data, off)
    raw = data[off:off+data_len]
    n = height * width
    pts = np.zeros((n,3), dtype=np.float32)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, pt_step)
    dt_sz = {1:1,2:1,3:2,4:2,5:4,6:4,7:4,8:8}
    for name, foff, dtype, _ in fields:
        if name not in ('x','y','z'): continue
        col = {'x':0,'y':1,'z':2}[name]; sz = dt_sz.get(dtype,0)
        if sz != 4: continue
        pts[:,col] = np.frombuffer(arr[:,foff:foff+sz].tobytes(), dtype=np.float32)
    valid = np.all(np.isfinite(pts), axis=1)
    d = np.linalg.norm(pts[valid], axis=1)
    valid[valid] &= (d>0.5)&(d<200)
    return stamp, pts[valid].astype(np.float64)

def parse_imu_cdr(data: bytes):
    """手动解析 sensor_msgs/Imu CDR → (stamp, gx,gy,gz, ax,ay,az)."""
    off = 0
    off += 4; sec,off=r_i32(data,off); nsec,off=r_u32(data,off)
    stamp = sec + nsec*1e-9
    _, off = r_string(data, off)
    off += 13*8
    gx,off=r_f64(data,off); gy,off=r_f64(data,off); gz,off=r_f64(data,off)
    off += 9*8
    ax,off=r_f64(data,off); ay,off=r_f64(data,off); az,off=r_f64(data,off)
    return stamp, np.array([gx,gy,gz]), np.array([ax,ay,az])

def parse_odom_cdr(data: bytes):
    """手动解析 nav_msgs/Odometry CDR → (stamp, pose_4x4, twist_linear, twist_angular)."""
    off = 0
    off += 4  # header.seq
    sec, off = r_i32(data, off); nsec, off = r_u32(data, off)
    stamp = sec + nsec*1e-9
    # frame_id
    slen, off = r_u32(data, off)
    off += (slen + 3) & ~3
    # child_frame_id
    slen, off = r_u32(data, off)
    off += (slen + 3) & ~3
    # pose: position + orientation
    px,off=r_f64(data,off); py,off=r_f64(data,off); pz,off=r_f64(data,off)
    ox,off=r_f64(data,off); oy,off=r_f64(data,off); oz,off=r_f64(data,off); ow,off=r_f64(data,off)
    T = np.eye(4)
    T[:3,3] = [px, py, pz]
    T[:3,:3] = R.from_quat([ox, oy, oz, ow]).as_matrix()
    off += 36 * 8  # pose covariance
    # twist
    vx,off=r_f64(data,off); vy,off=r_f64(data,off); vz,off=r_f64(data,off)
    wx,off=r_f64(data,off); wy,off=r_f64(data,off); wz,off=r_f64(data,off)
    return stamp, T, np.array([vx,vy,vz]), np.array([wx,wy,wz])

# ─── 数据加载 ───
def load_odom_from_bag(bag_dir: str):
    """从 sqlite3 加载腿部里程计 (50Hz)"""
    db = list(Path(bag_dir).glob("*.db3"))[0]
    conn = sqlite3.connect(str(db))
    cur = conn.cursor()
    cur.execute("SELECT id FROM topics WHERE name='/dog_odom'")
    row = cur.fetchone()
    if not row:
        conn.close()
        return []
    odom_id = row[0]
    cur.execute(f"SELECT data FROM messages WHERE topic_id={odom_id} ORDER BY timestamp")
    odoms = []
    for (data,) in cur:
        odoms.append(parse_odom_cdr(data))
    conn.close()
    print(f"  odom: {len(odoms)} msgs")
    return odoms

def load_from_bag(bag_dir: str):
    """sqlite3 直读 .db3, CDR 手动解析."""
    db = list(Path(bag_dir).glob("*.db3"))[0]
    conn = sqlite3.connect(str(db))
    cur = conn.cursor()
    cur.execute("SELECT id,name FROM topics")
    topics = {name:tid for tid,name in cur.fetchall()}
    lidar1_id = topics.get('/rslidar_points'); lidar2_id = topics.get('/livox/lidar')
    if not lidar1_id: raise RuntimeError("LiDAR1 topic not found")

    scans1, scans2 = [], []
    for tid, lst in [(lidar1_id, scans1), (lidar2_id, scans2)]:
        if tid is None: continue
        cur.execute(f"SELECT data FROM messages WHERE topic_id={tid} ORDER BY timestamp")
        for (data,) in cur:
            stamp, pts = parse_pc2_cdr(data)
            if len(pts) >= 50: lst.append((stamp, pts))
    conn.close()

    merged, j = [], 0
    for ts1, pts1 in scans1:
        while j<len(scans2) and scans2[j][0] < ts1-0.15: j += 1
        best_j, best_dt = j, float('inf')
        for k in range(j, min(j+3, len(scans2))):
            dt = abs(scans2[k][0]-ts1)
            if dt < best_dt: best_dt, best_j = dt, k
        pts2 = scans2[best_j][1] if best_dt<0.2 and best_j<len(scans2) else None

        pts1_imu = (T_L1_IMU[:3,:3] @ pts1.T).T + T_L1_IMU[:3,3]
        if pts2 is not None:
            pts2_imu = (T_L2_IMU[:3,:3] @ pts2.T).T + T_L2_IMU[:3,3]
            xyz = np.vstack([pts1_imu, pts2_imu])
        else:
            xyz = pts1_imu
        merged.append((ts1, xyz))
    print(f"  scans: {len(scans1)}+{len(scans2)} -> merged={len(merged)}")
    return merged

def load_from_pcds(pcd_dir: str):
    """加载 PCD 帧序列 (文件名格式: frame_NNNNNN_timestamp.pcd)."""
    files = sorted(Path(pcd_dir).glob("frame_*.pcd"))
    scans = []
    for f in files:
        pcd = o3d.io.read_point_cloud(str(f))
        xyz = np.asarray(pcd.points).astype(np.float64)
        ts = float(f.stem.split('_')[-1])
        scans.append((ts, xyz))
    print(f"  Loaded {len(scans)} PCD frames")
    return scans

# ─── 位姿处理 ───
def init_pose_from_yaml(pose_str: str) -> np.ndarray:
    import re
    nums = [float(x) for x in re.findall(r'[-+]?\d*\.?\d+', pose_str)]
    if len(nums) < 6:
        raise ValueError(f"init_pose needs 6 values [x,y,z,roll,pitch,yaw], got {len(nums)}: {nums}")
    x, y, z, roll, pitch, yaw = nums[:6]
    rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    return T

def tum_pose(ts, T):
    rot = np.array(T[:3,:3], copy=True); q = R.from_matrix(rot).as_quat()
    return f"{ts:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"

# ─── 腿部里程计 + GICP ───
def run_odom_gicp(scans, map_cloud, init_T, output_path, args):
    """腿部里程计帧间预测 + 帧到地图GICP修正

    用 B2 腿部里程计 (50Hz) 做帧间运动预测，替代不可靠的 GICP 帧间匹配。
    然后用帧到地图 GICP 修正漂移。
    """
    # 加载里程计
    odoms = load_odom_from_bag(args.bag if args.bag else args.pcd_dir)
    if not odoms:
        print("WARNING: No odom data, falling back to GICP")
        return run_gicp(scans, map_cloud, init_T, output_path, args)

    # 按时间排序
    odoms.sort(key=lambda x: x[0])

    crit = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=GICP_EPS, relative_rmse=GICP_EPS, max_iteration=GICP_ITER)
    gicp = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()

    def get_odom_delta(t1, t2):
        """从 t1 到 t2 的里程计增量 (在 t1 时刻的 body 系下)"""
        # 找到 t1 和 t2 最近的里程计消息
        i1 = min(range(len(odoms)), key=lambda i: abs(odoms[i][0] - t1))
        i2 = min(range(len(odoms)), key=lambda i: abs(odoms[i][0] - t2))
        if i1 == i2:
            return np.eye(4)

        T1 = odoms[i1][1]  # odom系下 t1 时刻的位姿
        T2 = odoms[i2][1]  # odom系下 t2 时刻的位姿
        # 相对运动: body1 → body2
        delta = np.linalg.inv(T1) @ T2
        return delta

    cur_T = init_T.copy()
    prev_scan_ts = scans[0][0] if scans else 0
    traj = []
    total = (len(scans) + args.every - 1) // args.every

    for fi, si in enumerate(range(0, len(scans), args.every)):
        ts, xyz = scans[si]

        # 帧到地图 GICP
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc = pc.voxel_down_sample(args.voxel_scan)
        if len(pc.points) < 50:
            traj.append(tum_pose(ts, cur_T))
            continue
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_scan*3, max_nn=30))

        # 用里程计预测当前帧位姿
        if fi == 0:
            guess = init_T
            prev_scan_ts = ts
        else:
            odom_delta = get_odom_delta(prev_scan_ts, ts)
            # 地面约束: 只取 XY + yaw
            rpy = R.from_matrix(odom_delta[:3,:3]).as_euler('xyz')
            odom_delta_2d = np.eye(4)
            odom_delta_2d[:3,:3] = R.from_euler('xyz', [0, 0, rpy[2]]).as_matrix()
            odom_delta_2d[:2, 3] = odom_delta[:2, 3]
            guess = cur_T @ odom_delta_2d

        reg = o3d.pipelines.registration.registration_generalized_icp(
            pc, map_cloud, GICP_CORR, guess, gicp, crit)

        if reg.fitness > 0.6:
            cur_T = np.copy(reg.transformation)
        else:
            # GICP失败时用里程计预测值（保持运动跟踪）
            cur_T = guess
            if reg.fitness > 0.3:
                # 若GICP有部分匹配, 融合: 取GICP的旋转 + 里程计的平移
                cur_T[:3,:3] = reg.transformation[:3,:3]

        prev_scan_ts = ts
        traj.append(tum_pose(ts, cur_T))

        if fi % 50 == 0:
            print(f"  [{fi+1}/{total}] t={ts:.1f}s fit={reg.fitness:.4f} "
                  f"odom_guess=({guess[0,3]:.1f},{guess[1,3]:.1f},{guess[2,3]:.1f}) "
                  f"gicp=({cur_T[0,3]:.1f},{cur_T[1,3]:.1f},{cur_T[2,3]:.1f})")

    Path(output_path).write_text("\n".join(traj) + "\n")
    print(f"\nOdom+GICP: {len(traj)} poses -> {output_path}")

# ─── Pose Graph Optimization ───
def run_pgo(scans, map_cloud, init_T, output_path, args):
    """因子图优化: 帧间里程计 + 地图先验 + 锚点

    边类型:
      - 里程计边 (binary): 帧 i-1 → 帧 i, 约束相对运动
      - 地图先验边 (unary): 世界原点 → 帧 i, 约束绝对位姿
      - 锚点边: 帧 0 固定在 init_T
    """
    from open3d.pipelines.registration import (
        PoseGraph, PoseGraphNode, PoseGraphEdge,
        global_optimization,
        GlobalOptimizationLevenbergMarquardt,
        GlobalOptimizationConvergenceCriteria,
        GlobalOptimizationOption,
    )

    total = (len(scans) + args.every - 1) // args.every
    gicp_est = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()

    crit_accurate = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-8, relative_rmse=1e-8, max_iteration=100)
    crit_fast = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-5, relative_rmse=1e-5, max_iteration=40)

    # 信息矩阵权重
    W_ODOM = 1000.0   # 里程计边权重 (极高: 相信帧间运动)
    W_PRIOR = 1.0     # 地图先验边权重 (很低: 仅修正漂移)
    W_ANCHOR = 10000.0  # 锚点边权重 (固定首帧)

    # ── Step 1: 预计算帧间里程计和地图先验 ──
    print(f"\nStep 1: Pre-computing {total} frames...")
    map_priors = []     # (T_map, fitness) for each frame
    odom_edges = []     # (T_rel, fitness) between consecutive frames
    odom_poses = []     # 累积里程计位姿 (作为节点初值)
    prev_scan = None
    cur_odom_T = init_T.copy()
    odom_poses.append(cur_odom_T.copy())

    for fi, si in enumerate(range(0, len(scans), args.every)):
        ts, xyz = scans[si]

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc = pc.voxel_down_sample(args.voxel_scan)
        if len(pc.points) < 50:
            map_priors.append((None, 0.0))
            odom_edges.append((None, 0.0))
            odom_poses.append(cur_odom_T.copy())
            prev_scan = None
            continue
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_scan*3, max_nn=30))

        # 帧到地图 GICP (先验) — 每帧独立配准，防止前一帧错误传播
        init_guess = init_T
        reg = o3d.pipelines.registration.registration_generalized_icp(
            pc, map_cloud, GICP_CORR * 2, init_guess, gicp_est, crit_accurate)
        if reg.fitness < 0.3:
            # 用里程计位姿做初值重试
            reg = o3d.pipelines.registration.registration_generalized_icp(
                pc, map_cloud, GICP_CORR * 5, cur_odom_T, gicp_est, crit_accurate)
        map_priors.append((np.copy(reg.transformation), reg.fitness))

        # 帧间 GICP (里程计) — 上一帧 → 当前帧
        if fi > 0 and prev_scan is not None:
            reg_odom = o3d.pipelines.registration.registration_generalized_icp(
                pc, prev_scan, GICP_CORR * 3, np.eye(4), gicp_est, crit_fast)
            T_rel = np.copy(reg_odom.transformation)
            odom_fit = reg_odom.fitness
            if odom_fit > 0.3:
                # 地面约束: 只取 XY 平移和 yaw 旋转, 固定 Z/roll/pitch
                rpy = R.from_matrix(T_rel[:3, :3]).as_euler('xyz')
                yaw = rpy[2]  # 只保留 yaw
                T_constrained = np.eye(4)
                T_constrained[:3, :3] = R.from_euler('xyz', [0, 0, yaw]).as_matrix()
                T_constrained[:2, 3] = T_rel[:2, 3]  # XY translation
                cur_odom_T = cur_odom_T @ T_constrained
        else:
            T_rel = np.eye(4)
            odom_fit = 1.0
        odom_edges.append((T_rel, odom_fit))
        odom_poses.append(cur_odom_T.copy())

        prev_scan = pc

        if fi % 50 == 0:
            print(f"  [{fi+1}/{total}] t={ts:.1f}s map_fit={reg.fitness:.3f} "
                  f"odom_fit={odom_fit:.3f} "
                  f"odom=({cur_odom_T[0,3]:.1f},{cur_odom_T[1,3]:.1f},{cur_odom_T[2,3]:.1f}) "
                  f"map=({reg.transformation[0,3]:.1f},{reg.transformation[1,3]:.1f},{reg.transformation[2,3]:.1f})")

    # ── Step 2: 构建 PoseGraph ──
    print(f"\nStep 2: Building PoseGraph...")
    pg = PoseGraph()

    def make_info(diag_weight):
        """创建对角的 6x6 信息矩阵"""
        info = np.eye(6) * diag_weight
        return info

    for i in range(total):
        # 用里程计累积位姿作为节点初值 (而非帧到地图 GICP)
        node_pose = odom_poses[i]
        node = PoseGraphNode(node_pose)
        pg.nodes.append(node)

        # 地图先验边: world(0) → node_i (仅当 fitness 好时, 低权重)
        T_map, fitness = map_priors[i]
        if T_map is not None and fitness > 0.7:
            edge_prior = PoseGraphEdge(
                0, i,
                T_map,
                make_info(W_PRIOR),
                uncertain=True
            )
            pg.edges.append(edge_prior)

        # 里程计边: node_{i-1} → node_i (高权重, 相信帧间运动)
        if i > 0:
            T_rel, odom_fit = odom_edges[i]
            if T_rel is not None and odom_fit > 0.5:
                edge_odom = PoseGraphEdge(
                    i - 1, i,
                    T_rel,
                    make_info(W_ODOM),
                    uncertain=False
                )
                pg.edges.append(edge_odom)

    # 锚点边: 固定 node 0
    # 用极高权重约束 node 0 → init_T
    anchor_edge = PoseGraphEdge(
        0, 0,  # self-loop at node 0
        init_T,
        make_info(W_ANCHOR),
        uncertain=False
    )
    pg.edges.append(anchor_edge)

    print(f"  Nodes: {len(pg.nodes)}, Edges: {len(pg.edges)}")

    # ── Step 3: 优化 ──
    print(f"\nStep 3: Optimizing pose graph...")
    method = GlobalOptimizationLevenbergMarquardt()
    criteria = GlobalOptimizationConvergenceCriteria()
    criteria.max_iteration = 200
    option = GlobalOptimizationOption(
        max_correspondence_distance=GICP_CORR,
        edge_prune_threshold=0.25,
        reference_node=0
    )

    global_optimization(pg, method, criteria, option)

    # ── Step 4: 提取优化后轨迹 ──
    print(f"\nStep 4: Writing trajectory...")
    lines = []
    for i, si in enumerate(range(0, len(scans), args.every)):
        if i >= len(pg.nodes):
            break
        ts = scans[si][0]
        pose = np.array(pg.nodes[i].pose)
        lines.append(tum_pose(ts, pose))

    Path(output_path).write_text("\n".join(lines) + "\n")
    print(f"  {len(lines)} poses -> {output_path}")

# ─── GICP 帧到地图配准 ───
def run_gicp(scans, map_cloud, init_T, output_path, args):
    """Open3D GICP 帧到地图配准"""
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=GICP_EPS, relative_rmse=GICP_EPS, max_iteration=GICP_ITER)
    gicp = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()

    prev_T = init_T.copy(); first = True; last_good_T = init_T.copy(); traj = []
    total = (len(scans) + args.every - 1) // args.every
    for fi, si in enumerate(range(0, len(scans), args.every)):
        ts, xyz = scans[si]

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc = pc.voxel_down_sample(args.voxel_scan)
        if len(pc.points) < 50: traj.append(tum_pose(ts, prev_T)); continue
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_scan*3, max_nn=30))

        init = init_T if first else last_good_T
        reg = o3d.pipelines.registration.registration_generalized_icp(
            pc, map_cloud, GICP_CORR, init, gicp, crit)

        if reg.fitness < 0.7 and not first:
            reg = o3d.pipelines.registration.registration_generalized_icp(
                pc, map_cloud, GICP_CORR * 2, init, gicp, crit)
        if reg.fitness < 0.4:
            traj.append(tum_pose(ts, last_good_T))
            if fi % 10 == 0:
                print(f"  [{fi+1}/{total}] t={ts:.1f}s fit={reg.fitness:.4f} REJECTED")
            continue

        last_good_T = np.copy(reg.transformation)
        first = False; prev_T = last_good_T
        traj.append(tum_pose(ts, prev_T))
        if fi % 10 == 0:
            print(f"  [{fi+1}/{total}] t={ts:.1f}s fit={reg.fitness:.4f} "
                  f"rmse={reg.inlier_rmse:.4f} pos=({prev_T[0,3]:.1f},{prev_T[1,3]:.1f},{prev_T[2,3]:.1f})")
    Path(output_path).write_text("\n".join(traj) + "\n")

# ─── main ───
def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--bag"); g.add_argument("--pcd_dir")
    p.add_argument("--map", required=True)
    p.add_argument("--output", default="gt_traj.txt")
    p.add_argument("--every", type=int, default=1)
    p.add_argument("--voxel", type=float, default=VOXEL_MAP)
    p.add_argument("--voxel_scan", type=float, default=VOXEL_SCAN)
    p.add_argument("--method", choices=['gicp', 'pgo', 'odom_gicp'], default='odom_gicp',
                   help='配准方法: gicp | pgo (因子图) | odom_gicp (腿部里程计+GICP, 默认)')
    p.add_argument("--init_pose", default=None,
                   help='初始位姿 [x,y,z,roll,pitch,yaw(deg)]')
    args = p.parse_args()

    print(f"Method: {args.method.upper()}")
    print(f"Map: {args.map}")
    mp = o3d.io.read_point_cloud(args.map)
    print(f"  {len(mp.points)} -> voxel({args.voxel}m)")
    mp = mp.voxel_down_sample(args.voxel)
    print(f"  -> {len(mp.points)}")
    mp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel*2, max_nn=30))

    if args.bag:
        scans = load_from_bag(args.bag)
    else:
        scans = load_from_pcds(args.pcd_dir)

    if args.init_pose:
        init_T = init_pose_from_yaml(args.init_pose)
        print(f"Init pose: t={init_T[:3,3]} rpy={R.from_matrix(init_T[:3,:3]).as_euler('xyz',degrees=True)}")
    else:
        init_T = np.eye(4)

    if args.method == 'odom_gicp':
        run_odom_gicp(scans, mp, init_T, args.output, args)
    elif args.method == 'pgo':
        run_pgo(scans, mp, init_T, args.output, args)
    else:
        run_gicp(scans, mp, init_T, args.output, args)

    print(f"Done -> {args.output}")

if __name__ == "__main__":
    main()
