#!/usr/bin/env python3
"""受控演示 步骤2：把扫描转到地图系，并用最近邻距离验证坐标变换正确。"""
import numpy as np, open3d as o3d
from scipy.spatial.transform import Rotation as R

SCAN="/home/creepert/Super-LIO/exp_controlled/scan_raw.npy"
POSE="/home/creepert/Super-LIO/exp_controlled/ref_pose.npy"
MAP ="/home/creepert/Super-LIO/src/super_lio/map/GroutingHole-DrainageHole.pcd"
T_LIDAR_IMU=np.array([0.34218,0.0,0.17851])   # b2.yaml lio.extrinsic.lidar_imu（正确值）

pts=np.load(SCAN); xyz=np.load(POSE)[:3]; q=np.load(POSE)[3:]
print(f"扫描 {len(pts)} 点 | 位姿 xyz={xyz.round(3)} | 四元数={q.round(6)} (norm={np.linalg.norm(q):.5f})")

# 点云 → IMU/body 系 → 地图系
p_body = pts + T_LIDAR_IMU                     # R_ext = I
p_map  = R.from_quat(q).apply(p_body) + xyz    # TUM 四元数顺序 (qx,qy,qz,qw)
print(f"地图系下扫描范围: x[{p_map[:,0].min():.1f},{p_map[:,0].max():.1f}] "
      f"y[{p_map[:,1].min():.1f},{p_map[:,1].max():.1f}] z[{p_map[:,2].min():.1f},{p_map[:,2].max():.1f}]")

print("\n加载地图（638万点，需降采样）…")
m = o3d.io.read_point_cloud(MAP)
print(f"地图原始 {len(m.points)} 点，bbox={m.get_axis_aligned_bounding_box()}")
# 只保留扫描附近 ±60 m 的地图区域，加速且更有针对性
P=np.asarray(m.points)
sel=(np.abs(P[:,0]-xyz[0])<60)&(np.abs(P[:,1]-xyz[1])<60)
Psub=P[sel]
print(f"扫描邻域内地图点 {len(Psub)}")
kd=o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Psub)))
d=np.array([kd.search_knn_vector_3d(p,1)[2][0] for p in p_map[::10]])**0.5
print(f"\n★ 验证：扫描点→地图最近邻距离（真值位姿，无偏移）")
print(f"  中位={np.median(d)*100:.2f} cm  P90={np.percentile(d,90)*100:.2f} cm  均值={d.mean()*100:.2f} cm")
print(f"  → 若中位在几 cm 量级，说明坐标系与位姿正确；若在米级，则变换有误")
np.save("/home/creepert/Super-LIO/exp_controlled/scan_map.npy", p_map)
