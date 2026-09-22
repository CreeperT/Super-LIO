#!/usr/bin/env python3
"""
轨迹可视化工具

用法:
  python3 tools/plot_traj.py traj/gt.txt                      # 单条轨迹
  python3 tools/plot_traj.py traj/gt.txt traj/lio.txt         # 对比两条
  python3 tools/plot_traj.py gt.txt -l "GT" lio.txt -l "LIO"  # 自定义图例
  python3 tools/plot_traj.py gt.txt --2d                       # 仅 2D 俯视图
  python3 tools/plot_traj.py gt.txt -o plot.png                # 保存到文件
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path


def load_tum(path):
    data = np.loadtxt(path)
    ts = data[:, 0]
    xyz = data[:, 1:4]
    quat = data[:, 4:]
    rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
    return ts, xyz, rpy


def main():
    p = argparse.ArgumentParser()
    p.add_argument("traj", nargs='+', help="TUM 格式轨迹文件")
    p.add_argument("-l", "--labels", nargs='+', default=None, help="图例标签 (与文件一一对应)")
    p.add_argument("--2d", action="store_true", help="仅 2D 俯视图")
    p.add_argument("--title", default="Trajectory")
    p.add_argument("-o", "--output", default=None, help="保存到文件 (png/pdf/svg)")
    args = p.parse_args()

    labels = args.labels if args.labels else [Path(f).stem for f in args.traj]
    if len(labels) != len(args.traj):
        labels = [Path(f).stem for f in args.traj]

    trajs = [load_tum(f) for f in args.traj]

    if args.output:
        plt.switch_backend('Agg')

    is_2d = getattr(args, '2d')  # --2d → attribute named '2d'
    if is_2d:
        fig, ax2d = plt.subplots(1, 1, figsize=(8, 8))
        fig.suptitle(args.title)
    else:
        fig = plt.figure(figsize=(18, 5))
        ax2d = fig.add_subplot(1, 3, 1)
        ax3d = fig.add_subplot(1, 3, 2, projection='3d')
        ax_yaw = fig.add_subplot(1, 3, 3)
        fig.suptitle(args.title)

    colors = plt.cm.tab10(np.linspace(0, 1, len(trajs)))

    # ── 2D 俯视图 ──
    for i, (ts, xyz, rpy) in enumerate(trajs):
        ax2d.plot(xyz[:, 0], xyz[:, 1], '-', color=colors[i], linewidth=0.5, label=labels[i])
        ax2d.scatter(xyz[0, 0], xyz[0, 1], c=[colors[i]], marker='o', s=40, zorder=5)
        ax2d.scatter(xyz[-1, 0], xyz[-1, 1], c=[colors[i]], marker='s', s=40, zorder=5)
    ax2d.set_xlabel('x (m)'); ax2d.set_ylabel('y (m)')
    ax2d.axis('equal'); ax2d.legend(); ax2d.grid(True, alpha=0.3)
    ax2d.set_title('Top-down (x-y)  ○=start  □=end')

    if not is_2d:
        # ── 3D ──
        for i, (ts, xyz, rpy) in enumerate(trajs):
            ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], '-', color=colors[i], linewidth=0.5, label=labels[i])
        ax3d.set_xlabel('x (m)'); ax3d.set_ylabel('y (m)'); ax3d.set_zlabel('z (m)')
        ax3d.legend(); ax3d.set_title('3D view')
        ax3d.view_init(elev=30, azim=-60)

        # ── yaw ──
        for i, (ts, xyz, rpy) in enumerate(trajs):
            t_min = (ts - ts[0]) / 60
            ax_yaw.plot(t_min, rpy[:, 2], '-', color=colors[i], linewidth=0.5, label=labels[i])
        ax_yaw.set_xlabel('time (min)'); ax_yaw.set_ylabel('yaw (°)')
        ax_yaw.legend(); ax_yaw.grid(True, alpha=0.3); ax_yaw.set_title('Yaw')

    plt.tight_layout()
    if args.output:
        plt.savefig(args.output, dpi=200, bbox_inches='tight')
        print(f"Saved → {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
