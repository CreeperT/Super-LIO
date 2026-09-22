#!/usr/bin/env python3
"""
将 ros2 bag 中的 /lio/odom 轨迹导出为 TUM 格式，供 evo 评估使用。

用法:
  python3 export_traj.py bag_file/ output_traj.txt

输出格式 (TUM):
  timestamp tx ty tz qx qy qz qw
"""

import sys
import os
import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from rosidl_runtime_py.utilities import get_message
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


def export_ros2(bag_dir: str, output_path: str, topic: str = "/lio/odom"):
    """从 ros2 bag 导出 odometry 轨迹"""
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 设置过滤
    from rosbag2_py import StorageFilter
    reader.set_filter(StorageFilter(topics=[topic]))

    poses = []
    twists = []
    while reader.has_next():
        (topic_name, data, stamp) = reader.read_next()
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        poses.append(f"{t:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}")
        # 速度状态：ESKF 的速度塌陷（沿程掉路程那一类失败）在这里可比位置早一步看见，
        # 且是与外部真值无关的内部信号（见 CHANGELOG 续 11/12）。
        tw = msg.twist.twist
        twists.append(f"{t:.9f} {tw.linear.x:.6f} {tw.linear.y:.6f} {tw.linear.z:.6f} "
                      f"{tw.angular.x:.6f} {tw.angular.y:.6f} {tw.angular.z:.6f}")

    with open(output_path, "w") as f:
        f.write("\n".join(poses))

    twist_path = os.path.splitext(output_path)[0] + "_twist.txt"
    with open(twist_path, "w") as f:
        f.write("\n".join(twists))

    print(f"Exported {len(poses)} poses from '{topic}' -> {output_path}")
    print(f"Exported {len(twists)} twists -> {twist_path}")


def export_ros1(bag_path: str, output_path: str, topic: str = "/lio/odom"):
    """从 ros1 bag 导出 odometry 轨迹（兼容）"""
    try:
        import rosbag
    except ImportError:
        print("需要 rosbag 包: pip install rosbag")
        sys.exit(1)

    bag = rosbag.Bag(bag_path)
    poses = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = t.to_sec()
        poses.append(f"{ts:.9f} {msg.pose.pose.position.x:.6f} "
                     f"{msg.pose.pose.position.y:.6f} {msg.pose.pose.position.z:.6f} "
                     f"{msg.pose.pose.orientation.x:.6f} {msg.pose.pose.orientation.y:.6f} "
                     f"{msg.pose.pose.orientation.z:.6f} {msg.pose.pose.orientation.w:.6f}")
    bag.close()

    with open(output_path, "w") as f:
        f.write("\n".join(poses))
    print(f"Exported {len(poses)} poses -> {output_path}")


def compute_loop_closure(traj_path: str, start_offset: float = 1.0, end_offset: float = 1.0):
    """
    计算闭环误差：比较轨迹开始 N 秒和结束 N 秒的平均位姿。
    适用于返程回到起点的 bag。
    """
    data = np.loadtxt(traj_path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[0] < 2:
        print("轨迹点太少，无法计算闭环误差")
        return

    timestamps = data[:, 0]
    t_start = timestamps[0]
    t_end = timestamps[-1]

    # 取开头 start_offset 秒的平均
    start_mask = timestamps <= t_start + start_offset
    start_traj = data[start_mask]
    start_pos = start_traj[:, 1:4].mean(axis=0)

    # 取结尾 end_offset 秒的平均
    end_mask = timestamps >= t_end - end_offset
    end_traj = data[end_mask]
    end_pos = end_traj[:, 1:4].mean(axis=0)

    loop_error = np.linalg.norm(end_pos - start_pos)
    print(f"\n===== 闭环误差 =====")
    print(f"起点平均位置: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
    print(f"终点平均位置: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")
    print(f"闭环误差: {loop_error:.3f}m")
    print(f"轨迹时长: {t_end - t_start:.1f}s")
    print(f"总里程: {np.sum(np.linalg.norm(np.diff(data[:, 1:4], axis=0), axis=1)):.1f}m")
    print(f"漂移率: {loop_error / (t_end - t_start) * 100:.3f} m/100s")
    return loop_error


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法:")
        print("  导出轨迹:   python3 export_traj.py <rosbag_dir> <output.txt> [topic]")
        print("  计算闭环:   python3 export_traj.py --loop <trajectory.txt>")
        print("  录包+导出:   ros2 bag record -o traj_out /lio/odom")
        sys.exit(1)

    if sys.argv[1] == "--loop":
        compute_loop_closure(sys.argv[2])
    elif HAS_ROS2:
        topic = sys.argv[3] if len(sys.argv) > 3 else "/lio/odom"
        export_ros2(sys.argv[1], sys.argv[2], topic)
    else:
        topic = sys.argv[3] if len(sys.argv) > 3 else "/lio/odom"
        export_ros1(sys.argv[1], sys.argv[2], topic)
