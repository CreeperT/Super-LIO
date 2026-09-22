#!/usr/bin/env python3
"""最小提取节点: 双 LiDAR + IMU → PCD 帧序列 (回调驱动, 无定时器)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np
import argparse
from pathlib import Path
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class BagExtractor(Node):
    def __init__(self, out_dir: str, every_n: int = 1):
        super().__init__('bag_extractor')
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.every_n = every_n

        self.lidar1 = None  # (stamp, xyz)
        self.lidar2 = None
        self.imu_list = []
        self.frame_no = 0
        self.saved = 0

        # 可靠的 QoS (匹配 bag 录制时的 QoS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_lidar2 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=256,
        )

        self.sub1 = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar1_cb, qos)
        self.sub2 = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar2_cb, qos_lidar2)
        self.sub_imu = self.create_subscription(Imu, '/dog_imu_raw', self.imu_cb, qos)

        # 延迟启动检查
        self.create_timer(5.0, self.start_check)
        self.checking = False

    def start_check(self):
        self.checking = True
        self.get_logger().info("Starting extraction check loop")

    def parse_xyz(self, msg: PointCloud2):
        n = msg.width * msg.height
        pts = np.zeros((n, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        for f in msg.fields:
            if f.name not in ('x', 'y', 'z') or f.datatype != 7:  # FLOAT32
                continue
            col = {'x': 0, 'y': 1, 'z': 2}[f.name]
            pts[:, col] = np.frombuffer(raw[:, f.offset:f.offset+4].tobytes(), dtype=np.float32)
        valid = np.all(np.isfinite(pts), axis=1)
        d = np.linalg.norm(pts[valid], axis=1)
        valid[valid] &= (d > 0.5) & (d < 200.0)
        return pts[valid]

    def lidar1_cb(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar1 = (ts, self.parse_xyz(msg))
        self.try_save()

    def lidar2_cb(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar2 = (ts, self.parse_xyz(msg))

    def imu_cb(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_list.append((ts, msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z, msg.linear_acceleration.x,
                              msg.linear_acceleration.y, msg.linear_acceleration.z))

    def try_save(self):
        if not self.checking or self.lidar1 is None:
            return
        self.frame_no += 1
        if self.frame_no % self.every_n != 0:
            return

        ts1, pts1 = self.lidar1
        # 合并 LiDAR2
        if self.lidar2 is not None:
            ts2, pts2 = self.lidar2
            if abs(ts1 - ts2) < 0.2 and len(pts2) > 0:
                pts_all = np.vstack([pts1, pts2])
            else:
                pts_all = pts1
        else:
            pts_all = pts1

        if len(pts_all) < 100:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_all.astype(np.float64))
        fname = self.out_dir / f"frame_{self.saved:06d}_{ts1:.6f}.pcd"
        o3d.io.write_point_cloud(str(fname), pcd)
        self.saved += 1
        if self.saved % 100 == 0:
            self.get_logger().info(f"Saved {self.saved} frames")

        # 每个 LiDAR1 只保存一次
        self.lidar1 = None


def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/pcd_frames")
    parser.add_argument("--every", type=int, default=1)
    args = parser.parse_args()
    node = BagExtractor(args.out, args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Done. Saved {node.saved} frames")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
