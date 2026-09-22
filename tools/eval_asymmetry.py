#!/usr/bin/env python3
"""
不对称性验证脚本
用法:
  python3 eval_asymmetry.py base_traj.txt compare_traj.txt [--label "配置名"]
  python3 eval_asymmetry.py reloc_traj.txt slam_calib_off.txt --label "标定OFF"
  python3 eval_asymmetry.py reloc_traj.txt slam_calib_on.txt --label "标定ON"

功能:
  1. 按折返点将轨迹切分为 去程 / 返程
  2. 分别计算两段的 ATE
  3. 输出不对称性指标：ATE_ratio = ATE_返程 / ATE_去程
     ratio > 1 表示存在不对称漂移（返程比去程差）
"""

import sys
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


def load_tum(path):
    d = np.loadtxt(path)
    ts = d[:, 0]
    xyz = d[:, 1:4]
    quat = d[:, 4:]
    return ts, xyz, quat


def sync_trajectories(ts_a, xyz_a, ts_b, xyz_b, max_dt=0.2):
    """按时间戳匹配两轨迹，返回 (xyz_a_matched, xyz_b_matched, ts_matched)"""
    matched_a, matched_b, matched_ts = [], [], []
    j = 0
    for i in range(len(ts_a)):
        t = ts_a[i]
        while j < len(ts_b) and ts_b[j] < t - max_dt:
            j += 1
        if j < len(ts_b) and abs(ts_b[j] - t) <= max_dt:
            matched_a.append(xyz_a[i])
            matched_b.append(xyz_b[j])
            matched_ts.append(t)
    return np.array(matched_a), np.array(matched_b), np.array(matched_ts)


def compute_umeyama_alignment(src, dst, use_2d=False):
    """Umeyama 7-DOF 对齐，返回对齐后的 src. 若 use_2d, 仅对齐 x,y 平面."""
    if use_2d:
        # 仅用 x,y 做3DOF对齐 (x, y, yaw)
        src_xy = src[:, :2]; dst_xy = dst[:, :2]
        c_s = src_xy.mean(axis=0); c_d = dst_xy.mean(axis=0)
        H = (src_xy - c_s).T @ (dst_xy - c_d)
        U, _, Vt = np.linalg.svd(H)
        R_2d = Vt.T @ U.T
        if np.linalg.det(R_2d) < 0:
            Vt[-1, :] *= -1; R_2d = Vt.T @ U.T
        t_2d = c_d - R_2d @ c_s
        aligned = src.copy()
        aligned[:, :2] = (R_2d @ src_xy.T).T + t_2d
        return aligned
    else:
        centroid_src = src.mean(axis=0)
        centroid_dst = dst.mean(axis=0)
        src_c = src - centroid_src
        dst_c = dst - centroid_dst
        H = src_c.T @ dst_c
        U, _, Vt = np.linalg.svd(H)
        R_align = Vt.T @ U.T
        if np.linalg.det(R_align) < 0:
            Vt[-1, :] *= -1
            R_align = Vt.T @ U.T
        t_align = centroid_dst - R_align @ centroid_src
        return (R_align @ src.T).T + t_align


def compute_ate(xyz_est, xyz_ref):
    """计算 ATE RMSE"""
    diffs = np.linalg.norm(xyz_est - xyz_ref, axis=1)
    return {
        'rmse': np.sqrt(np.mean(diffs ** 2)),
        'mean': np.mean(diffs),
        'median': np.median(diffs),
        'max': np.max(diffs),
        'std': np.std(diffs),
        'n': len(diffs),
    }


def find_turning_point(ts, xyz):
    """找折返点: x 最小 或 y 最大 的时刻（选距离起点最远的点）"""
    start = xyz[0]
    dists = np.linalg.norm(xyz - start, axis=1)
    return np.argmax(dists)


def evaluate_asymmetry(ref_path, est_path, label="", split_ts=None, trim_first=0, trim_last=0,
                       use_2d=False, max_dt=0.3, align=False, trim_sec=0.0):
    """
    评估不对称性。

    参数:
        ref_path:  参考轨迹 (如 reloc_traj)
        est_path:  待评估轨迹 (如 slam_traj)
        label:     实验标签
        split_ts:  手动指定的拆分时间戳（None=自动找折返点）
        align:     是否先做全局 Umeyama 对齐。仅当 est 与 ref 坐标系不同时才需要
                   (如 SLAM 轨迹 vs 重定位参考); 同一先验地图系的 reloc 轨迹应保持
                   False — 对齐会吸收部分往返偏移, 且被启动段离群点污染。
        trim_sec:  剔除轨迹前 N 秒 (启动 ICP 对准瞬态; B 段建议 30)

    返回:
        dict: 含 outbound ATE, return ATE, 不对称性比率等
    """
    ts_r, xyz_r, _ = load_tum(ref_path)
    ts_e, xyz_e, _ = load_tum(est_path)

    # 同步 (时间插值; 最近邻同步会在帧抖动处注入 ±1 帧错位伪差 ~0.1 m)
    xyz_r_sync, xyz_e_sync, ts_sync = interp_sync_trajectories(ts_r, xyz_r, ts_e, xyz_e, max_gap=max_dt)
    print(f"  Interpolated frames: {len(xyz_r_sync)} / {len(ts_e)}")

    # 找折返点
    if split_ts is None:
        turn_idx = find_turning_point(ts_r, xyz_r)
        split_ts = ts_r[turn_idx]

    t_min_sync = (ts_sync - ts_r[0]) / 60
    split_min = (split_ts - ts_r[0]) / 60
    print(f"  Split at t={split_min:.1f}min (timestamp={split_ts:.3f})")

    # 剔除启动段 (秒, 相对参考首帧)
    if trim_sec > 0:
        keep = ts_sync - ts_r[0] >= trim_sec
        xyz_r_sync = xyz_r_sync[keep]
        xyz_e_sync = xyz_e_sync[keep]
        ts_sync = ts_sync[keep]
        print(f"  Trimmed first {trim_sec:.0f}s (init transient) -> {len(ts_sync)} frames remaining")

    # 砍掉首尾帧 (帧数, 与 trim_sec 叠加)
    if trim_first > 0 or trim_last > 0:
        end_idx = len(xyz_r_sync) - trim_last if trim_last > 0 else len(xyz_r_sync)
        xyz_r_sync = xyz_r_sync[trim_first:end_idx]
        xyz_e_sync = xyz_e_sync[trim_first:end_idx]
        ts_sync = ts_sync[trim_first:end_idx]
        print(f"  Trimmed: first {trim_first}, last {trim_last} -> {len(ts_sync)} frames remaining")

    # 全局 Umeyama 对齐（仅异坐标系轨迹需要; 用全部同步帧计算变换）
    mode = ("2D" if use_2d else "3D") + (" Umeyama" if align else " raw")
    print(f"  Evaluation mode: {mode}")
    if align:
        xyz_e_sync = compute_umeyama_alignment(xyz_e_sync, xyz_r_sync, use_2d)
    elif use_2d:
        xyz_r_sync[:, 2] = 0
        xyz_e_sync[:, 2] = 0

    # 切分去程和返程 (用同步后的时间戳)
    out_mask = ts_sync <= split_ts
    ret_mask = ts_sync >= split_ts
    print(f"  去程: {out_mask.sum()}帧, 返程: {ret_mask.sum()}帧")

    # 2D模式: 仅用 x,y 计算ATE (Z置零)
    if use_2d:
        xyz_r_sync[:, 2] = 0
        xyz_e_sync[:, 2] = 0

    # 分别计算 ATE
    ate_all = compute_ate(xyz_e_sync, xyz_r_sync)
    ate_out = compute_ate(xyz_e_sync[out_mask], xyz_r_sync[out_mask])
    ate_ret = compute_ate(xyz_e_sync[ret_mask], xyz_r_sync[ret_mask])

    raw_ratio = ate_ret['rmse'] / ate_out['rmse'] if ate_out['rmse'] > 0 else float('inf')
    asymmetry_ratio = max(raw_ratio, 1.0 / raw_ratio) if raw_ratio > 0 else float('inf')

    # 输出
    print(f"\n  {'='*60}")
    print(f"  {label or Path(est_path).stem}")
    print(f"  {'='*60}")
    print(f"  {'':>12} {'Frames':>8} {'RMSE':>8} {'Mean':>8} {'Median':>8} {'Max':>8}")
    print(f"  {'全程':>12} {ate_all['n']:>8} {ate_all['rmse']:>8.3f} {ate_all['mean']:>8.3f} "
          f"{ate_all['median']:>8.3f} {ate_all['max']:>8.3f}")
    print(f"  {'去程':>12} {ate_out['n']:>8} {ate_out['rmse']:>8.3f} {ate_out['mean']:>8.3f} "
          f"{ate_out['median']:>8.3f} {ate_out['max']:>8.3f}")
    print(f"  {'返程':>12} {ate_ret['n']:>8} {ate_ret['rmse']:>8.3f} {ate_ret['mean']:>8.3f} "
          f"{ate_ret['median']:>8.3f} {ate_ret['max']:>8.3f}")
    print(f"  {'---':>12} {'---':>8} {'---':>8}")
    print(f"  {'不对称率(max)':>12} {'':>8} {asymmetry_ratio:>8.3f}  (max(返/去, 去/返))")

    return {
        'label': label,
        'split_ts': split_ts,
        'ate_all': ate_all,
        'ate_out': ate_out,
        'ate_ret': ate_ret,
        'asymmetry_ratio': asymmetry_ratio,
    }


def main():
    import argparse
    p = argparse.ArgumentParser(description='往返不对称性验证')
    p.add_argument('ref', help='参考轨迹 (如 reloc_traj.txt)')
    p.add_argument('est', nargs='+', help='待评估轨迹 (可多个)')
    p.add_argument('--label', '-l', nargs='+', default=[], help='实验标签')
    p.add_argument('--split_ts', type=float, default=None, help='手动拆分时间戳')
    p.add_argument('--split_file', default=None, help='从文件读取拆分时间戳')
    p.add_argument('--trim_first', type=int, default=0, help='跳过前N帧 (标定收敛期)')
    p.add_argument('--trim_last', type=int, default=0, help='跳过后N帧')
    p.add_argument('--trim_sec', type=float, default=0.0,
                   help='剔除轨迹前N秒 (启动ICP对准瞬态; 同坐标系reloc建议30)')
    p.add_argument('--align', action='store_true',
                   help='全局Umeyama对齐 (仅坐标系不同的轨迹需要, 如SLAM vs reloc参考)')
    p.add_argument('--2d', action='store_true', help='2D评估 (仅x,y, 忽略Z)')
    p.add_argument('--max_dt', type=float, default=0.3,
                   help='插值同步最大时间间隔(s)，默认0.3')
    args = p.parse_args()

    # 从文件读拆分时间戳（避免每次自动检测不一致）
    split_ts = args.split_ts
    if args.split_file:
        with open(args.split_file) as f:
            split_ts = float(f.read().strip())
        print(f"Split timestamp from file: {split_ts:.3f}")

    # 先确定拆分点（用第一条轨迹自动检测）
    if split_ts is None:
        ts_r, xyz_r, _ = load_tum(args.ref)
        turn_idx = find_turning_point(ts_r, xyz_r)
        split_ts = ts_r[turn_idx]
        print(f"Auto-detected split timestamp: {split_ts:.3f}")
        # 保存到文件
        split_path = args.ref.replace('.txt', '_split.txt')
        Path(split_path).write_text(f"{split_ts:.6f}\n")
        print(f"Saved to: {split_path}")

    labels = args.label if args.label else [Path(f).stem for f in args.est]

    results = []
    for est_path, label in zip(args.est, labels):
        r = evaluate_asymmetry(args.ref, est_path, label, split_ts,
                              trim_first=args.trim_first, trim_last=args.trim_last,
                              use_2d=getattr(args, '2d'), max_dt=args.max_dt,
                              align=args.align, trim_sec=args.trim_sec)
        results.append(r)

    # 汇总对比
    if len(results) > 1:
        print(f"\n\n  {'='*60}")
        print(f"  汇总对比")
        print(f"  {'='*60}")
        print(f"  {'配置':<20} {'去程RMSE':>10} {'返程RMSE':>10} {'不对称率':>10} {'判定':>12}")
        for r in results:
            ratio = r['asymmetry_ratio']
            if ratio > 2.0:
                verdict = "⚠ 显著不对称"
            elif ratio > 1.3:
                verdict = "⚡ 轻微不对称"
            else:
                verdict = "✅ 对称"
            print(f"  {r['label']:<20} {r['ate_out']['rmse']:>10.3f} {r['ate_ret']['rmse']:>10.3f} "
                  f"{ratio:>10.3f} {verdict:>12}")


# ────────────────────────────────────────────────────────────
# [2026 新增] 时间插值同步 + 双口径评估
# 最近邻同步(max_dt=0.2)在帧抖动处产生 ±1 帧错位, 注入 ~0.1m 纵向伪差
# (est=ref 自比 0.13m → 插值后精确 0)。改用: 时间戳去重 + 线性插值。
# ────────────────────────────────────────────────────────────
def interp_sync_trajectories(ts_r, xyz_r, ts_e, xyz_e, max_gap=0.3):
    """参考与估计两侧均去重后, est 按 ref 时间戳线性插值;
    返回 (xyz_r_m, xyz_e_m, ts_m). 参考端不去重会在重复时间戳帧处
    引入伪差 (B 段参考开头 7 个重复启动帧 → 自比 0.15 m)."""
    ts_r_u, jr = np.unique(ts_r, return_index=True)
    xyz_r_u = xyz_r[jr]
    ts_e_u, jx = np.unique(ts_e, return_index=True)
    xyz_e_u = xyz_e[jx]
    idx = np.searchsorted(ts_e_u, ts_r_u)
    idx = np.clip(idx, 1, len(ts_e_u) - 1)
    t0e, t1e = ts_e_u[idx - 1], ts_e_u[idx]
    dt = np.minimum(np.abs(ts_r_u - t0e), np.abs(t1e - ts_r_u))
    valid = dt < max_gap
    w = np.clip((ts_r_u - t0e) / np.maximum(t1e - t0e, 1e-9), 0, 1)
    est = xyz_e_u[idx - 1] + w[:, None] * (xyz_e_u[idx] - xyz_e_u[idx - 1])
    return xyz_r_u[valid], est[valid], ts_r_u[valid]


def evaluate_dual(ref_path, est_path, label="", align=False, split_ts=None):
    """插值同步双口径评估: 返回 (raw 去/返 RMSE, raw 比, align 去/返, align 比,
    raw 去/返 median, 去/返帧数)"""
    ts_r, xyz_r, _ = load_tum(ref_path)
    ts_e, xyz_e, _ = load_tum(est_path)
    if split_ts is None:
        split_ts = ts_r[find_turning_point(ts_r, xyz_r)]
    a, b, ts = interp_sync_trajectories(ts_r, xyz_r, ts_e, xyz_e)
    if align:
        b = compute_umeyama_alignment(b, a, False)
    mo, mr = ts <= split_ts, ts >= split_ts
    e = np.linalg.norm(b - a, axis=1)
    ro, rr = np.sqrt((e[mo] ** 2).mean()), np.sqrt((e[mr] ** 2).mean())
    mdo, mdr = np.median(e[mo]), np.median(e[mr])
    if not align:
        b2 = compute_umeyama_alignment(b, a, False)
        e2 = np.linalg.norm(b2 - a, axis=1)
        ao, ar = np.sqrt((e2[mo] ** 2).mean()), np.sqrt((e2[mr] ** 2).mean())
    else:
        ao, ar = ro, rr
    r_ratio = max(rr / ro, ro / rr) if ro > 0 else float('inf')
    a_ratio = max(ar / ao, ao / ar) if ao > 0 else float('inf')
    return dict(label=label, ro=ro, rr=rr, r_ratio=r_ratio, ao=ao, ar=ar,
                a_ratio=a_ratio, mdo=mdo, mdr=mdr, n_out=mo.sum(), n_ret=mr.sum())


if __name__ == '__main__':
    main()
