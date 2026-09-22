#!/usr/bin/env python3
"""无对齐仲裁：把某轮估计轨迹的**路程**与 /dog_odom（腿部里程计）及参考逐窗口对比。

为什么需要：`dev`（返程↔去程最近距离）对纵向盲、`rms_*` 需要对齐、而单次 3D Umeyama
会把**一次性台阶**摊成整条轨迹的常值偏移（r10：全程对齐后读作"前 10 min 稳定 −4.4 m"，
按去程单独对齐则是 +0.16 m / 尾段 −20.8 m）。路程是**零对齐**量，腿部里程计是自带真值。

用法:
  python3 tools/check_dog_odom.py exp_repeat/aslam_graft_r10 exp_repeat/aslam_grade566_r1
  python3 tools/check_dog_odom.py --all 'exp_repeat/aslam_graft_r*'
  python3 tools/check_dog_odom.py --dump-only          # 只刷新 /dog_odom 缓存
"""
import argparse
import pathlib
import sys

import numpy as np

WS = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / 'tools'))
from eval_asymmetry import load_tum                       # noqa: E402

BAG = pathlib.Path.home() / '下载/rosbag/5#GJD'
CACHE = WS / 'exp_repeat/dog_odom.txt'                    # /dog_odom 的 TUM 缓存
REF = WS / 'traj/reloc_traj.txt'
WIN = [(0, 100), (100, 200), (200, 300), (300, 428), (428, 500),
       (500, 600), (600, 700), (700, 800), (800, 817)]
FLAG = -3.0                                               # 窗口路程亏损告警阈值（m）


def dump_dog_odom():
    """读 bag 里的 /dog_odom 落成 TUM（与参考同一 bag 时间轴）。"""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = SequentialReader()
    r.open(StorageOptions(uri=str(BAG), storage_id='sqlite3'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/dog_odom']))
    n = 0
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    with CACHE.open('w') as f:
        while r.has_next():
            _, data, _ = r.read_next()
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                    f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
            n += 1
    print(f'/dog_odom → {CACHE}（{n} 条）')


def dedup(t, x):
    k = np.concatenate([[True], np.diff(t) > 0])
    return t[k], x[k]


def path_len(t, xyz, t0, a, b):
    m = (t - t0 >= a) & (t - t0 <= b)
    if m.sum() < 2:
        return float('nan')
    return float(np.linalg.norm(np.diff(xyz[m][:, :2], axis=0), axis=1).sum())


def gap_table(dirs, t0, ref, win):
    """窗口内"记录缺口 vs 路程亏损"：把"亏损是录制缺口造成的"直接排除掉。

    位姿若在窗内连续（缺口只占零点几秒）却仍少走路程 ⇒ 位姿存在但不前进 ⇒ 滤波器欠积分，
    与录制/输出链路无关（r6 实测：窗内仅缺 1.5 s，却亏 13.8 m）。
    """
    print(f"\n=== {win[0]}–{win[1]} s 窗：缺口 vs 亏损 ===")
    print(f"{'run':<26}{'窗内点数':>9}{'最大间隔':>9}{'>0.5s处数':>10}{'窗内缺失':>9}{'窗内亏损':>10}")
    for d in dirs:
        t, x = dedup(*load_tum(str(d / 'traj.txt'))[:2])
        m = (t - t0 >= win[0]) & (t - t0 <= win[1])
        if m.sum() < 2:
            continue
        dt = np.diff(t)[m[:-1]]
        miss = float(dt[dt > 0.5].sum())
        seg = path_len(t, x, t0, *win)
        print(f'{d.name:<26}{int(m.sum()):>9}{float(dt.max()):>9.3f}{int((dt > 0.5).sum()):>10}'
              f'{miss:>9.1f}{seg - ref[win]:>+10.1f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='*')
    ap.add_argument('--all', help='glob，如 exp_repeat/aslam_graft_r*')
    ap.add_argument('--dump-only', action='store_true')
    ap.add_argument('--gaps', action='store_true', help='附"窗口缺口 vs 亏损"表')
    ap.add_argument('--win', type=float, nargs=2, default=[600, 700], help='--gaps 的窗口（bag 秒）')
    a = ap.parse_args()

    if not CACHE.exists() or a.dump_only:
        dump_dog_odom()
    if a.dump_only:
        return

    rt, rxyz, _ = load_tum(str(REF))
    t0 = rt[0]
    rt, rxyz = dedup(rt, rxyz)
    dt_, dxyz = dedup(*load_tum(str(CACHE))[:2])
    ref = {w: path_len(rt, rxyz, t0, *w) for w in WIN}
    dog = {w: path_len(dt_, dxyz, t0, *w) for w in WIN}

    dirs = [pathlib.Path(d) for d in a.dirs]
    if a.all:
        dirs += sorted(WS.glob(a.all))
    dirs = [d for d in dirs if (d / 'traj.txt').exists()]
    if not dirs:
        sys.exit('未找到含 traj.txt 的目录')

    print('窗口路程（水平，m）；est−ref 为负 = 估计器掉路程。dog 列自证"狗有没有停"。')
    print(f"{'':<26}" + ''.join(f'{x:>6}-{y:<5}' for x, y in WIN))
    print(f"{'参考':<26}" + ''.join(f'{ref[w]:>11.1f}' for w in WIN))
    print(f"{'/dog_odom 腿部里程计':<26}" + ''.join(f'{dog[w]:>11.1f}' for w in WIN))
    for d in dirs:
        t, x = dedup(*load_tum(str(d / 'traj.txt'))[:2])
        dif = [path_len(t, x, t0, *w) - ref[w] for w in WIN]
        worst = np.nanmin(dif)
        print(f'{d.name:<26}' + ''.join(f'{v:>+11.1f}' for v in dif)
              + (f'  ← 窗口掉路程 {worst:.1f} m' if worst < FLAG else ''))

    if a.gaps:
        gap_table(dirs, t0, ref, (a.win[0], a.win[1]))


if __name__ == '__main__':
    main()
