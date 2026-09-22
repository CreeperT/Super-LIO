#!/usr/bin/env python3
"""受控演示 步骤1：从 A 段 bag 提取分岔点附近一帧 rslidar 扫描，
用参考轨迹的真值位姿 + 外参转到地图坐标系，存为 pcd。"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
import open3d as o3d

BAG="/home/creepert/下载/rosbag/5#GJD"
REF="/home/creepert/Super-LIO/traj/reloc_traj.txt"
T_BAG0=1782872338.507        # bag 首帧 header.stamp
T_TARGET=T_BAG0+636.0        # 分岔点 t=636 s
# 前雷达外参（b2.yaml: lio.extrinsic.lidar_imu，正确值）
T_LIDAR_IMU = np.array([0.34218, 0.0, 0.17851])
R_LIDAR_IMU = np.eye(3)

def read_scan():
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("",""))
    best=None; bestdt=1e9
    while r.has_next():
        tp,data,t_recv=r.read_next()
        if tp!="/rslidar_points": continue
        m=deserialize_message(data, PointCloud2)
        ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        dt=abs(ts-T_TARGET)
        if dt<bestdt: bestdt=dt; best=(o3d.t.io.read_point_cloud(data=m) if False else parse(m))
        if ts>T_TARGET+2: break
    return best, bestdt

def parse(m):
    # PointCloud2 → Nx3（只取 x,y,z；字段顺序 intensity,rgb,x,y,z）
    n=m.width*m.height
    buf=np.frombuffer(m.data, dtype=np.uint8).reshape(n, m.point_step)
    off={f.name: f.offset for f in m.fields}
    xs=np.frombuffer(buf[:, off['x']:off['x']+4].tobytes(), dtype=np.float32)
    ys=np.frombuffer(buf[:, off['y']:off['y']+4].tobytes(), dtype=np.float32)
    zs=np.frombuffer(buf[:, off['z']:off['z']+4].tobytes(), dtype=np.float32)
    p=np.stack([xs,ys,zs],1).astype(np.float64)
    return p[np.isfinite(p).all(1)]

def ref_pose(t):
    d=np.loadtxt(REF); ts=d[:,0]; xyz=d[:,1:4]; q=d[:,4:8]   # TUM: x y z qx qy qz qw? 需确认
    i=np.argmin(np.abs(ts-t))
    return ts[i], xyz[i], q[i]

if __name__=="__main__":
    pts,dt=read_scan()
    print(f"提取扫描：{len(pts)} 点，与目标时刻差 {dt:.3f}s")
    ts,xyz,q=ref_pose(T_TARGET)
    print(f"参考位姿 @ t={ts-T_BAG0:.1f}s: xyz={xyz}")
    print(f"四元数(原始前4列)={q}")
    np.save("/home/creepert/Super-LIO/exp_controlled/scan_raw.npy", pts)
    np.save("/home/creepert/Super-LIO/exp_controlled/ref_pose.npy", np.concatenate([xyz,q]))
    print("已存 scan_raw.npy / ref_pose.npy")
