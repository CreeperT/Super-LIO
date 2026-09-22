#!/usr/bin/env python3
"""`logs/structure.csv` 的分段统计（判据卡 2.3）：逐段报告 `grade_resid` 的 mean/MAD/|p95|。

为什么要分段: 该文件是 **append 模式**，跨轮累积、无表头，以"帧号回退"为段界。实测 461 202 行中
461 142 行是 11 列（现行格式），另 **60 行为撕裂/残留**（10 列 1 行、12–21 列共 59 行，多来自 SIGKILL
时的不完整写）⇒ 必须**跳过并计数**，不能硬解析。

口径:
  · 段界 = frame_num 回退处；段内只保留 11 列且全数值可解析的行
  · 列（0-based）: 0 frame, 1 snap_wall, 2 dtheta_wall, 3 fit_ok, 4 fit_fail, 5 cls_ground,
    6 cls_wall, 7 cls_mid, 8 ground_inliers, **9 grade_resid_deg**, 10 valid
  · `grade_resid` 只统计 **valid=1** 的行（通道未激活的帧恒为 0，会拉平均值）
  · 稳健统计用 **MAD**（1.4826 × median|dev| ≈ σ）：std 会被离群行污染、跨臂不可比
  · 判据卡 2.3 阈值: |mean| ≤ 0.15°、MAD ≤ 0.30°、|p95| ≤ 0.70°
    （实测好轮 opt566: mean −0.077°、MAD 0.161、|p95| 0.397；另有健康段 p95 = 0.610 ⇒ 卡里早先的
    |mean| ≤ 0.05° 与 |p95| ≤ 0.60° 都会把健康轮判失败，已改）
  · 顺带给出 ★ wall 通道的逐段事实: `snap_wall == cls_wall` 的行占比、`dtheta_wall` 的 p95

用法:
  python3 tools/structure_stats.py                 # 全部段，末 12 段打表
  python3 tools/structure_stats.py --last 3        # 只看最后 3 段
  python3 tools/structure_stats.py --csv /tmp/s.csv
"""
import argparse
import pathlib

import numpy as np

CSV = pathlib.Path('/home/creepert/Super-LIO/src/super_lio/logs/structure.csv')
# 判据卡 2.3 阈值
LIM_MEAN, LIM_MAD, LIM_P95 = 0.15, 0.30, 0.70


def load(path):
    """按帧号回退分段；返回 (段列表, 跳过的行数, 总行数)。"""
    segs, cur, junk, total = [], [], 0, 0
    last = None
    for ln in path.open(errors='ignore'):
        total += 1
        p = ln.strip().split(',')
        if len(p) != 11:
            junk += 1
            continue
        try:
            r = [float(v) for v in p]
        except ValueError:
            junk += 1
            continue
        if last is not None and r[0] < last:
            if cur:
                segs.append(np.array(cur))
            cur = []
        cur.append(r)
        last = r[0]
    if cur:
        segs.append(np.array(cur))
    return segs, junk, total


def seg_stats(s):
    a = s[s[:, 10] == 1]
    g = a[:, 9]
    if len(g) < 10:
        return None
    med = float(np.median(g))
    return {'rows': len(s), 'act': len(g), 'f0': int(s[0, 0]), 'f1': int(s[-1, 0]),
            'mean': float(g.mean()), 'mad': 1.4826 * float(np.median(np.abs(g - med))),
            'p95': float(np.percentile(np.abs(g), 95)),
            'snap_eq': 100.0 * float(np.mean(s[:, 1] == s[:, 6])),
            'dth': float(np.percentile(s[:, 2], 95))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--last', type=int, default=12, help='只打最后 N 段（默认 12）')
    ap.add_argument('--csv', help='把逐段统计写到该 csv')
    ap.add_argument('--file', default=str(CSV))
    a = ap.parse_args()

    segs, junk, total = load(pathlib.Path(a.file))
    rows = [(i, seg_stats(s)) for i, s in enumerate(segs)]
    rows = [(i, st) for i, st in rows if st]
    print(f"{a.file}: 总 {total} 行 | 段数 {len(segs)} | 跳过 {junk} 行（撕裂/历史残留）")
    print(f"判据卡 2.3: |mean| ≤ {LIM_MEAN}°、MAD ≤ {LIM_MAD}°、|p95| ≤ {LIM_P95}°")
    print(f"{'段':>4}{'帧范围':>16}{'行数':>8}{'激活':>8}{'mean':>9}{'MAD':>8}{'|p95|':>8}"
          f"{'snap==cls':>11}{'dth p95':>9}  判定")
    for i, st in rows[-a.last:]:
        ok = (abs(st['mean']) <= LIM_MEAN) and (st['mad'] <= LIM_MAD) and (st['p95'] <= LIM_P95)
        print(f"{i:>4}{st['f0']:>8}→{st['f1']:<7}{st['rows']:>8}{st['act']:>8}"
              f"{st['mean']:>+9.4f}{st['mad']:>8.3f}{st['p95']:>8.3f}"
              f"{st['snap_eq']:>10.1f}%{st['dth']:>9.2f}"
              f"  {'通过' if ok else '**不通过**'}")

    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('seg,f0,f1,rows,active,mean_deg,mad_deg,p95_deg,snap_eq_pct,dtheta_p95_deg\n')
            for i, st in rows:
                f.write(f"{i},{st['f0']},{st['f1']},{st['rows']},{st['act']},"
                        f"{st['mean']:.4f},{st['mad']:.4f},{st['p95']:.4f},"
                        f"{st['snap_eq']:.1f},{st['dth']:.2f}\n")


if __name__ == '__main__':
    main()
