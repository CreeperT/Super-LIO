#!/usr/bin/env python3
"""
GTSAM 因子图拉平：消去 SLAM 轨迹的 Z 轴下坠

原理:
  - Odometry 边：SLAM 轨迹的相邻帧相对位姿 → 保形（局部形状不变）
  - Prior 边：稀疏锚点帧的帧到地图 GICP 结果 → 拉平（绝对位姿约束）
  - GTSAM 优化：平衡两者，输出平滑且绝对精准的轨迹

用法:
  python3 pgo_gtsam.py traj/slam_3_traj.txt \
    --bag ~/下载/rosbag/5#GJD \
    --map src/super_lio/map/GroutingHole-DrainageHole.pcd \
    --output traj/gt_gtsam.txt \
    --anchor_every 50
"""

import argparse, sqlite3, struct, sys
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import gtsam

# ─── 复用 generate_gt 的数据加载 ───
T_L1_IMU = np.eye(4); T_L1_IMU[:3,3] = [0.34218, 0.0, 0.17851]
T_L2_IMU = np.eye(4)
T_L2_IMU[:3,:3] = [[0.866025, 0.0, 0.5],[0.0, 1.0, 0.0],[-0.5, 0.0, 0.866025]]
T_L2_IMU[:3,3] = [-0.3, 0.0, 0.15]
GICP_CORR=3.0; GICP_ITER=200; VOXEL_MAP=0.5; VOXEL_SCAN=0.3

def r_u32(d,o): return struct.unpack_from('<I',d,o)[0], o+4
def r_i32(d,o): return struct.unpack_from('<i',d,o)[0], o+4
def r_string(d,o):
    L, o = r_u32(d, o)
    return d[o:o+L].split(b'\x00')[0].decode('utf-8','replace'), o + ((L+3)&~3)

def parse_pc2_cdr(data: bytes):
    off = 0; off += 4
    sec, off = r_i32(data, off); nsec, off = r_u32(data, off)
    stamp = sec + nsec*1e-9
    _, off = r_string(data, off)
    height, off = r_u32(data, off); width, off = r_u32(data, off)
    nf, off = r_u32(data, off)
    fields = []
    for _ in range(nf):
        name, off = r_string(data, off)
        foff, off = r_u32(data, off)
        dtype = data[off]; off += 1
        off = (off + 3) & ~3
        count, off = r_u32(data, off)
        fields.append((name, foff, dtype, count))
    off += 1; off = (off + 3) & ~3
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

# ─── 转换函数 ───
def T_to_gtsam_pose(T):
    """numpy 4x4 -> gtsam.Pose3"""
    rot = gtsam.Rot3(T[:3,:3])
    trans = gtsam.Point3(T[0,3], T[1,3], T[2,3])
    return gtsam.Pose3(rot, trans)

def gtsam_pose_to_T(pose):
    """gtsam.Pose3 -> numpy 4x4"""
    T = np.eye(4)
    T[:3,:3] = pose.rotation().matrix()
    T[:3,3] = pose.translation()
    return T

def tum_pose(ts, T):
    rot = np.array(T[:3,:3], copy=True); q = R.from_matrix(rot).as_quat()
    return f"{ts:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"

def load_slam_traj(path):
    d = np.loadtxt(path)
    ts = d[:, 0]
    xyz = d[:, 1:4]
    quat = d[:, 4:]
    # 重建 4x4 矩阵
    poses = []
    for i in range(len(d)):
        T = np.eye(4)
        T[:3,:3] = R.from_quat(quat[i]).as_matrix()
        T[:3,3] = xyz[i]
        poses.append(T)
    return ts, poses

def load_scan_at_timestamp(bag_dir, target_ts, max_dt=0.05):
    """从 bag 中加载最接近 target_ts 的双雷达合并扫描"""
    db = list(Path(bag_dir).glob("*.db3"))[0]
    conn = sqlite3.connect(str(db))
    cur = conn.cursor()
    cur.execute("SELECT id,name FROM topics")
    topics = {name:tid for tid,name in cur.fetchall()}
    lidar1_id = topics.get('/rslidar_points')
    lidar2_id = topics.get('/livox/lidar')

    def get_nearest(topic_id, target):
        # Bag 时间戳是纳秒整型, target 是秒 (float)
        target_ns = int(target * 1e9)
        window_ns = int(0.2 * 1e9)  # 200ms 窗口
        cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={topic_id} "
                    f"AND timestamp BETWEEN {target_ns - window_ns} AND {target_ns + window_ns} "
                    f"ORDER BY ABS(timestamp - {target_ns}) LIMIT 1")
        row = cur.fetchone()
        if row:
            return parse_pc2_cdr(row[1])
        return None, None

    r1 = get_nearest(lidar1_id, target_ts)
    r2 = get_nearest(lidar2_id, target_ts) if lidar2_id else (None, None)
    conn.close()

    if r1[1] is None:
        return None

    pts1_imu = (T_L1_IMU[:3,:3] @ r1[1].T).T + T_L1_IMU[:3,3]
    if r2[1] is not None:
        pts2_imu = (T_L2_IMU[:3,:3] @ r2[1].T).T + T_L2_IMU[:3,3]
        return np.vstack([pts1_imu, pts2_imu])
    return pts1_imu


def main():
    p = argparse.ArgumentParser(description='GTSAM 因子图拉平 Z 轴下坠')
    p.add_argument('slam_traj', help='SLAM 轨迹 (TUM格式, 含Z下坠)')
    p.add_argument('--bag', required=True, help='rosbag2 目录 (用于提取锚点帧扫描)')
    p.add_argument('--map', required=True, help='先验高精地图 (.pcd)')
    p.add_argument('--output', default='gt_gtsam.txt')
    p.add_argument('--anchor_every', type=int, default=50, help='每隔多少帧设一个锚点 (默认50)')
    p.add_argument('--anchor_min_fitness', type=float, default=0.6, help='锚点GICP最低fitness (默认0.6)')
    p.add_argument('--anchor_max_count', type=int, default=30, help='锚点最大数量 (默认30)')
    p.add_argument('--odom_sigma_xy', type=float, default=0.05, help='里程计XY噪声sigma (m, 默认0.05)')
    p.add_argument('--odom_sigma_z', type=float, default=2.0, help='里程计Z噪声sigma (m, 默认2.0, 高=不信任SLAM的Z)')
    p.add_argument('--prior_sigma', type=float, default=0.10, help='先验噪声sigma (m, 默认0.10)')
    p.add_argument('--z_prior_sigma', type=float, default=0.05, help='Z轴先验sigma (m, 默认0.05)')
    p.add_argument('--skip_first_n', type=int, default=0, help='跳过前N帧不设锚点 (静止段)')
    p.add_argument('--z_min', type=float, default=42.0, help='可接受的地图Z最小值 (默认42m)')
    p.add_argument('--z_max', type=float, default=50.0, help='可接受的地图Z最大值 (默认50m)')
    p.add_argument('--init_pose', default=None,
                   help='初始位姿 [x,y,z,roll,pitch,yaw(deg)], 用于SLAM→Map粗对齐')
    args = p.parse_args()

    # 粗对齐: SLAM局部坐标系 → 地图坐标系
    if args.init_pose:
        import re
        nums = [float(x) for x in re.findall(r'[-+]?\d*\.?\d+', args.init_pose)]
        T_slam_to_map = np.eye(4)
        T_slam_to_map[:3,:3] = R.from_euler('xyz', [nums[3], nums[4], nums[5]], degrees=True).as_matrix()
        T_slam_to_map[:3,3] = nums[:3]
    else:
        T_slam_to_map = np.eye(4)
    print(f"  SLAM→Map alignment: t={T_slam_to_map[:3,3]} rpy={R.from_matrix(T_slam_to_map[:3,:3]).as_euler('xyz',degrees=True)}")

    # ── Step 1: 加载 SLAM 轨迹 + 提取 odometry 边 ──
    print(f"Step 1: Loading SLAM trajectory...")
    ts, poses = load_slam_traj(args.slam_traj)
    print(f"  Frames: {len(ts)}, duration: {(ts[-1]-ts[0])/60:.1f}min")

    odom_edges = []
    for i in range(1, len(poses)):
        T_rel = np.linalg.inv(poses[i-1]) @ poses[i]
        odom_edges.append(T_rel)
    print(f"  Odometry edges: {len(odom_edges)}")

    print(f"  Odometry edges: {len(odom_edges)}")

    # ── Step 2: 选择锚点 + 帧到地图 GICP ──
    print(f"\nStep 2: Selecting anchor frames + frame-to-map GICP...")
    print(f"  Loading map...")
    mp = o3d.io.read_point_cloud(args.map)
    mp = mp.voxel_down_sample(VOXEL_MAP)
    mp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_MAP*2, max_nn=30))
    print(f"  Map: {len(mp.points)} points (voxel {VOXEL_MAP}m)")

    gicp = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=GICP_ITER)

    # 选锚点: 均匀空间采样 + 跳过静止段
    anchor_indices = list(range(args.skip_first_n, len(poses), args.anchor_every))
    if len(anchor_indices) > args.anchor_max_count:
        anchor_indices = anchor_indices[::len(anchor_indices)//args.anchor_max_count]

    print(f"  Anchor candidates: {len(anchor_indices)} frames")

    anchors = {}  # {idx: (gtsam_pose, fitness)}
    n_tried, n_accepted = 0, 0
    for idx in anchor_indices:
        if n_accepted >= args.anchor_max_count:
            break
        n_tried += 1
        xyz = load_scan_at_timestamp(args.bag, ts[idx])
        if xyz is None or len(xyz) < 50:
            continue

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc = pc.voxel_down_sample(VOXEL_SCAN)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SCAN*3, max_nn=30))

        # 用 SLAM 位姿粗对齐到地图坐标系后做 GICP 初值
        # 粗对齐: SLAM位姿 → 地图系 (Z可能偏离20m)
        init_guess = T_slam_to_map @ poses[idx]

        # 修正Z: 用地图中 (x,y) 附近的平均 Z 替代 SLAM 的 Z
        map_pts = np.asarray(mp.points)
        xy_guess = init_guess[:2,3]
        nearby = np.linalg.norm(map_pts[:,:2] - xy_guess, axis=1) < 5.0
        if nearby.sum() > 10:
            map_z_nearby = map_pts[nearby, 2].mean()
            init_guess[2,3] = map_z_nearby  # 修正Z到地图期望值

        # 分级搜索: 粗(30m) → 中(10m) → 精(3m)
        reg = None
        for corr_mult in [10.0, 5.0, 1.0]:
            reg = o3d.pipelines.registration.registration_generalized_icp(
                pc, mp, GICP_CORR * corr_mult, init_guess, gicp, crit)
            if reg.fitness > 0.4:
                init_guess = reg.transformation  # 用当前结果作下级初值
            if reg.fitness >= args.anchor_min_fitness:
                break

        gicp_z = reg.transformation[2,3]
        # Z范围过滤: 地图Z在44-48m, 排除匹配到错误楼层的锚点
        if reg.fitness >= args.anchor_min_fitness and args.z_min <= gicp_z <= args.z_max:
            anchors[idx] = (T_to_gtsam_pose(reg.transformation), reg.fitness)
            n_accepted += 1
            if n_accepted % 3 == 0 or n_accepted <= 3:
                slam_z_aligned = (T_slam_to_map @ poses[idx])[2,3]
                gicp_z = reg.transformation[2,3]
                print(f"  Anchor #{n_accepted} @ f{idx} fit={reg.fitness:.3f} "
                      f"gicp_z={gicp_z:.2f} slam_z={slam_z_aligned:.2f} "
                      f"Δz={gicp_z - slam_z_aligned:+.1f}m")

    print(f"  Accepted: {n_accepted}/{n_tried} anchor candidates")

    # ── Step 3: 构建 GTSAM 因子图 ──
    print(f"\nStep 3: Building GTSAM factor graph...")

    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # 噪声模型
    # 里程计: Z轴sigma远大于XY — SLAM的Z不可靠, 由锚点决定
    sigma_r = np.deg2rad(0.1)  # 旋转噪声 (0.1°/frame)
    sigma_t_xy = args.odom_sigma_xy  # XY平移噪声 (低=信任)
    sigma_t_z = args.odom_sigma_z    # Z平移噪声 (高=不信任)
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
        [sigma_r, sigma_r, sigma_r, sigma_t_xy, sigma_t_xy, sigma_t_z]))

    # 先验: Z轴比XY更确定 (地图已知地面高度)
    prior_noise_xy = args.prior_sigma
    prior_noise_z = args.z_prior_sigma
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
        [0.01, 0.01, 0.01, prior_noise_xy, prior_noise_xy, prior_noise_z]))

    # 初始化节点 (用 SLAM 位姿)
    for i, pose_T in enumerate(poses):
        initial.insert(i, T_to_gtsam_pose(pose_T))

    # 首帧 strong prior
    first_anchor_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
        [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]))
    if 0 in anchors:
        graph.add(gtsam.PriorFactorPose3(0, anchors[0][0], first_anchor_noise))
    else:
        graph.add(gtsam.PriorFactorPose3(0, T_to_gtsam_pose(poses[0]), first_anchor_noise))

    # Odometry 边 (XY可信, Z高容忍)
    for i, T_rel in enumerate(odom_edges):
        graph.add(gtsam.BetweenFactorPose3(i, i+1, T_to_gtsam_pose(T_rel), odom_noise))

    # Prior 边 (锚点)
    for idx, (gtsam_pose, fitness) in anchors.items():
        if idx == 0:
            continue  # 首帧已添加
        # 按 fitness 加权: 高 fitness = 低 sigma
        weight = max(0.5, fitness)
        anchor_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
            [0.01, 0.01, 0.01,
             prior_noise_xy / weight, prior_noise_xy / weight, prior_noise_z / weight]))
        graph.add(gtsam.PriorFactorPose3(idx, gtsam_pose, anchor_noise))

    print(f"  Graph: {graph.nrFactors()} factors, {initial.size()} variables")

    # ── Step 4: 优化 ──
    print(f"\nStep 4: Optimizing...")
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(200)
    params.setRelativeErrorTol(1e-8)
    params.setAbsoluteErrorTol(1e-8)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    print(f"  Iterations: {optimizer.iterations()}")
    print(f"  Final error: {optimizer.error():.2f}")

    # ── Step 5: 输出轨迹 ──
    print(f"\nStep 5: Writing optimized trajectory...")
    lines = []
    for i in range(len(poses)):
        opt_pose = result.atPose3(i)
        opt_T = gtsam_pose_to_T(opt_pose)
        lines.append(tum_pose(ts[i], opt_T))

    Path(args.output).write_text("\n".join(lines) + "\n")

    # 统计 Z 轴改善 (在 map 坐标系下)
    z_before_map = np.array([(T_slam_to_map @ poses[i])[2,3] for i in range(len(poses))])
    z_after = np.array([gtsam_pose_to_T(result.atPose3(i))[2,3] for i in range(len(poses))])

    print(f"\n  Z-axis correction summary (map frame):")
    print(f"    SLAM aligned:   [{z_before_map.min():.2f}, {z_before_map.max():.2f}] "
          f"range={z_before_map.ptp():.2f}m")
    print(f"    GTSAM optimized: [{z_after.min():.2f}, {z_after.max():.2f}] "
          f"range={z_after.ptp():.2f}m")
    print(f"    Target (map):    Z≈44~48m")

    # 锚点处的 Z 误差
    z_errors = []
    for idx in sorted(anchors.keys()):
        gicp_z = gtsam_pose_to_T(anchors[idx][0])[2,3]
        opt_z = gtsam_pose_to_T(result.atPose3(idx))[2,3]
        z_errors.append(abs(gicp_z - opt_z))
    if z_errors:
        print(f"    Anchor Z residual: mean={np.mean(z_errors):.3f}m max={np.max(z_errors):.3f}m")

    print(f"\nDone -> {args.output}")


if __name__ == '__main__':
    main()
