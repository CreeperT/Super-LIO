#!/usr/bin/env python3
"""重复性评测的扩展指标（graft/结构/标定各臂共用）。

**口径**（固定写明，便于跨轮审计）:
  - 同步: 参考与估计均按时间戳去重后线性插值（与 eval_asymmetry.interp_sync_trajectories 同源）
  - 对齐: 3D Umeyama，**无尺度**（与 eval_asymmetry --align 同源）
  - 起点偏置: Δz(t) = z_est − z_ref 后扣除 t₀ 处的值 ⇒ 只留形状误差
  - 径向位移: s = |xy(t) − xy(t₀)|（**原点参照**，往返路线上与路程口径差别极大）
  - 倾角: arctan( polyfit(s, 目标量, 1)[0] )，单位度
      · tilt(z)  = 估计轨迹自身相对原点的倾斜
      · tilt(Δz) = **误差**的倾斜；正=过陡、负=压平。二者恰差真值自身的径向斜率
  - rms_along / rms_cross: 对齐后误差**按参考轨迹局部行进方向投影**（±6 s 弦）后再取 RMS
      · 与地图系轴无关，是"沿程/横向"的规范口径
      · 本数据集的地图系 y 轴距行进方向 8.8°，故 rms_y ≈ 沿程、rms_x ≈ 横向，
        但两者相差 15–20%（轴倾斜把 1.3 m 的横向误差混进了沿程列）——**不要用 rms_x/rms_y 读方向**
      · 折返点附近该口径病态（弦跨越 ±180° 转向）
  - |ba|: 取 launch.log 最后一条 `[Bias#N]`（末态加速度计零偏模长）

用法:
  python3 tools/eval_ext.py exp_repeat/aslam_graft_r1 [--ref traj/reloc_traj.txt] [--csv]
  python3 tools/eval_ext.py --header > exp_repeat/<臂>_ext.csv   # 表头由本脚本统一产出
"""
import argparse
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from eval_asymmetry import load_tum                       # noqa: E402

SPLIT_5GJD = 1782872767.704156          # 5#GJD 折返点（bag 时间轴）
REF_GRADE_DEG = 0.566                   # 5#GJD 真值净径向坡度（重定位轨迹 z~径向位移斜率）
MAX_GAP_SEC = 0.5                       # 覆盖率缺口阈值
BIAS_RE = re.compile(r'\[Bias#\d+\] ba=\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\]')
GRADE_RE = re.compile(r'Ground channel active: reference\s+longitudinal grade = ([-\d.]+) deg')
# CSV 列（唯一出处；表头由 `--header` 产出，行由 `--csv` 产出，禁止在别处手写）
COLS = ['rmse', 'coverage', 'span', 'frames', 'z_range', 'tilt_z', 'tilt_dz', 'dz_med',
        'dz_max', 'dz_end', 'dz_al_rms', 'rms_x', 'rms_y', 'rms_z', 'rms_along',
        'rms_cross', 'vmin', 'dev_med', 'dev_p95', 'ba_norm', 'grade_ref', 'cap_tilt',
        'grade_err']
# vmin 的窗口：本数据集里"速度塌陷"型失败固定发生在 600–700 s（路线 ≈550–650 m 的返程段）。
# 折返点在 ≈428 s，故该窗内没有合法的停车。干净轮 min|v_xy| ≥ 0.65 m/s，失败轮 ≤ 0.04 m/s。
VMIN_WIN = (600.0, 700.0)
CAPTURE_RE = re.compile(r'Terrain normal captured from \d+ plane samples over \d+ frames:'
                        r'\s+tilt from vertical = ([-\d.]+) deg')


def umeyama3(A, B):
    """无尺度 3D Umeyama（H = src_cᵀ·dst_c），与 eval_asymmetry 同式。"""
    cA, cB = A.mean(0), B.mean(0)
    Ac, Bc = A - cA, B - cB
    U, _, Vt = np.linalg.svd(Ac.T @ Bc)
    Vt = Vt.copy()
    if np.linalg.det(Vt.T @ U.T) < 0:
        Vt[-1] *= -1
    R = Vt.T @ U.T
    return R, cB - R @ cA


def ref_heading(t, xy, win=6.0):
    """参考轨迹的局部行进方向（±win 秒弦，单位向量）。

    折返点附近病态：弦跨越 ±180° 转向时航向会在 1 帧内翻转，沿程分量随之跳变
    （r10 实测该处单帧跳变 8.19 m，排除后 0.67 m）——读沿程跳变前先排除 |Δ航向|>5° 的样本。
    """
    hd = np.empty_like(xy)
    for k, tk in enumerate(t):
        m = (t >= tk - win) & (t <= tk + win)
        j0, j1 = int(np.argmax(m)), len(m) - 1 - int(np.argmax(m[::-1]))
        if j1 - j0 < 3:
            j0, j1 = 0, len(t) - 1
        v = xy[j1] - xy[j0]
        n = np.linalg.norm(v)
        hd[k] = v / n if n > 1e-9 else [1.0, 0.0]
    return hd


def path_deviation(t, xyz, split):
    """返程每点到去程 (x,y) 折线的最近距离（点到线段）。"""
    m = t < split
    o, r = xyz[m, :2], xyz[~m, :2]
    if len(o) < 10 or len(r) < 10:
        return None
    step = max(1, len(o) // 3000)
    A, B = o[::step][:-1], o[::step][1:]
    ab = B - A
    L2 = np.einsum('ij,ij->i', ab, ab)
    L2[L2 < 1e-12] = 1e-12
    d = []
    for q in r[::max(1, len(r) // 3000)]:
        ap = q - A
        tt = np.clip(np.einsum('ij,ij->i', ap, ab) / L2, 0, 1)
        d.append(np.min(np.linalg.norm(q - (A + tt[:, None] * ab), axis=1)))
    return np.array(d)


def final_bias(d):
    f = pathlib.Path(d) / 'launch.log'
    if not f.exists():
        return None
    last = None
    for ln in f.open(errors='ignore'):
        m = BIAS_RE.search(ln)
        if m:
            last = np.array([float(m.group(i)) for i in (1, 2, 3)])
    return last


def structure_refs(d):
    """地面通道的参考纵向坡度与捕获到的地形法向倾角（都来自 launch.log 的一次性打印）。"""
    f = pathlib.Path(d) / 'launch.log'
    if not f.exists():
        return None, None
    grade = cap = None
    for ln in f.open(errors='ignore'):
        m = GRADE_RE.search(ln)
        if m:
            grade = float(m.group(1))
        m = CAPTURE_RE.search(ln)
        if m:
            cap = float(m.group(1))
    return grade, cap


def analyse(d, ref_path, split):
    d = pathlib.Path(d)
    ts, xyz, _ = load_tum(str(d / 'traj.txt'))
    rt, rxyz, _ = load_tum(ref_path)

    # 时间插值同步（估计端去重后再插值）
    keep = np.concatenate([[True], np.diff(ts) > 0])
    ts, xyz = ts[keep], xyz[keep]
    ref = np.column_stack([np.interp(ts, rt, rxyz[:, k]) for k in range(3)])

    est0, ref0 = xyz - xyz[0], ref - ref[0]
    R, t = umeyama3(est0, ref0)
    aligned = (R @ est0.T).T + t

    # Δz 用**对齐前**口径：z 误差的绝对值（含倾斜），扣起点偏置只去掉任意常值。
    # 注意不能对齐后再扣 dz[0]——Umeyama 已居中，再扣会把平移重新塞回去
    # （aslam_r2 实测：对齐后中位 2.19 → 误扣起点后 6.47）。
    dz = est0[:, 2] - ref0[:, 2]
    dz = dz - dz[0]
    s = np.linalg.norm(est0[:, :2], axis=1)                    # 径向位移（原点参照）
    tilt_z = np.degrees(np.arctan(np.polyfit(s, est0[:, 2], 1)[0]))
    tilt_dz = np.degrees(np.arctan(np.polyfit(s, dz, 1)[0]))
    # 对齐后残差（供诊断，与门控 RMSE 同源）
    dz_al = aligned[:, 2] - ref0[:, 2]
    # 对齐后各轴 RMS：`dev`（返程↔去程横向最近距离）**对纵向误差盲**，必须并列各轴
    ax_rms = [float(np.sqrt(np.mean((aligned[:, k] - ref0[:, k]) ** 2))) for k in range(3)]
    # 沿程/横向：按参考行进方向投影（不依赖地图系轴约定）
    hd = ref_heading(ts, ref0[:, :2])
    eh = aligned[:, :2] - ref0[:, :2]
    along = np.einsum('ij,ij->i', eh, hd)
    cross = np.einsum('ij,ij->i', eh, np.column_stack([-hd[:, 1], hd[:, 0]]))

    out = {
        'dir': str(d), 'frames': len(ts), 'span': float(ts[-1] - ts[0]),
        'z_range': float(xyz[:, 2].max() - xyz[:, 2].min()),
        'tilt_z': float(tilt_z), 'tilt_dz': float(tilt_dz),
        'dz_med': float(np.median(np.abs(dz))), 'dz_max': float(np.abs(dz).max()),
        'dz_end': float(dz[-1]),
        'dz_al_rms': float(np.sqrt(np.mean(dz_al ** 2))),
        'rms_x': ax_rms[0], 'rms_y': ax_rms[1], 'rms_z': ax_rms[2],
        'rms_along': float(np.sqrt(np.mean(along ** 2))),
        'rms_cross': float(np.sqrt(np.mean(cross ** 2))),
    }
    dt = np.diff(ts)
    out['coverage'] = 100.0 * (1.0 - dt[dt > MAX_GAP_SEC].sum() / out['span'])

    dev = path_deviation(ts, xyz, split)
    out['dev_med'] = float(np.median(dev)) if dev is not None else None
    out['dev_p95'] = float(np.percentile(dev, 95)) if dev is not None else None

    ev = d / 'eval.log'
    out['rmse'] = None
    if ev.exists():
        m = re.search(r'全程\s+\d+\s+([\d.]+)', ev.read_text(errors='ignore'))
        if m:
            out['rmse'] = float(m.group(1))

    # 速度塌陷（内部信号，无需外部真值）：读 export_traj.py 顺带导出的 twist
    tw = d / 'traj_twist.txt'
    out['vmin'] = None
    if tw.exists():
        a = np.loadtxt(str(tw))
        if a.ndim == 1:
            a = a.reshape(1, -1)
        m = (a[:, 0] - ts[0] >= VMIN_WIN[0]) & (a[:, 0] - ts[0] <= VMIN_WIN[1])
        if m.sum():
            out['vmin'] = float(np.hypot(a[m, 1], a[m, 2]).min())

    ba = final_bias(d)
    out['ba_norm'] = float(np.linalg.norm(ba)) if ba is not None else None
    out['ba'] = ba

    # 地面通道参考（决定剩余 z 误差的标量，见 CHANGELOG 2026-09-21 续 4）
    grade, cap = structure_refs(d)
    out['grade_ref'] = grade
    out['cap_tilt'] = cap
    # 相对真值净径向坡度的偏差与对应预测误差（真值 +0.566°，|Δz| ≈ 3.4 × 偏差）
    out['grade_err'] = None if grade is None else grade - REF_GRADE_DEG
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', nargs='?')
    ap.add_argument('--ref', default='traj/reloc_traj.txt')
    ap.add_argument('--split', type=float, default=SPLIT_5GJD)
    ap.add_argument('--csv', action='store_true')
    ap.add_argument('--header', action='store_true', help='只打印 CSV 表头（列清单的唯一出处）')
    a = ap.parse_args()
    if a.header:
        print('dir,' + ','.join(COLS))
        return

    r = analyse(a.dir, a.ref, a.split)
    if a.csv:
        vals = ['' if r[c] is None else f"{r[c]:.4f}" for c in COLS]
        print(f"{r['dir']}," + ','.join(vals))
        return
    f = lambda k, fmt='.3f': '—' if r[k] is None else format(r[k], fmt)   # noqa: E731
    print(f"{r['dir']}")
    print(f"  口径: 插值同步(去重) | 3D Umeyama 无尺度 | 起点偏置已扣 | 径向位移=|xy−xy₀|")
    print(f"  RMSE {f('rmse','.3f')}  覆盖率 {f('coverage','.2f')}%  跨度 {f('span','.0f')}s  帧数 {r['frames']}")
    print(f"  z 范围 {f('z_range','.2f')} m | 倾角 z(自身) {f('tilt_z','+.2f')}° "
          f"| 倾角 Δz(误差, 正=过陡/负=压平) {f('tilt_dz','+.2f')}°")
    print(f"  |Δz|(对齐前) 中位 {f('dz_med','.2f')} max {f('dz_max','.2f')} 末值 {f('dz_end','+.2f')}"
          f" | 对齐后 z RMS {f('dz_al_rms','.2f')}")
    print(f"  对齐后沿程/横向 RMS: 沿程 {f('rms_along','.2f')} 横向 {f('rms_cross','.2f')}"
          f"（按参考行进方向投影；折返点附近口径病态）")
    print(f"  对齐后各轴 RMS(参考地图系): x {f('rms_x','.2f')} y {f('rms_y','.2f')} z {f('rms_z','.2f')}"
          f"  ← 地图系轴，本数据集 y 距行进方向 8.8°，**不可当方向列读**（dev 则对纵向盲）")
    print(f"  返程偏离去程路径 中位 {f('dev_med','.2f')} P95 {f('dev_p95','.2f')}")
    print(f"  速度塌陷 vmin(600–700s min|v_xy|) {f('vmin','.3f')} m/s"
          f"  ← 干净轮 ≥0.65，塌陷轮 ≤0.04（零重叠）")
    print(f"  |ba| 末态 {f('ba_norm','.4f')}")
    print(f"  地面参考 grade_ref {f('grade_ref','.3f')}°  捕获法向倾角 {f('cap_tilt','.3f')}°"
          f"  | 偏离真值({REF_GRADE_DEG}°) {f('grade_err','+.3f')}° ⇒ 预期 |Δz| ≈ "
          f"{'—' if r['grade_err'] is None else format(3.4*abs(r['grade_err']), '.2f')} m")


if __name__ == '__main__':
    main()
