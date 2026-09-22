#!/usr/bin/env python3
"""重复性实验的单轮有效性门控（判据预先声明，机械可复核，非事后挑选）。

四项判据（全部满足才算有效）：
  ① 收敛：全程 RMSE < 1.0 m（发散运行可达 1e5 m 量级）
  ② 初始化成功：全程无 ESKF 退化抑制事件（`suppressed weakest direction`）
     —— 实测判据参照值与健康运行均为 0；初始化失配的那次为 17，
        且轨迹在 t=4.5 s 即偏离参照 >1 m，属启动阶段丢失地图匹配。
  ③ 记录完整：缺口（相邻帧间隔 > 0.5 s）总和 < 时间跨度的 5%
     —— 论文现用的那次为 23%，不通过。
  ④ 速度塌陷（判据卡 1.0，2026-09-22 增补）：窗口内 min|v_xy|（取自 /lio/odom 的 twist）> 阈值。
     实测 23 轮**零重叠**：干净 0.620–0.689，塌陷 0.004–0.041 m/s。
     **这是第 1 层唯一对"沿程分岔"有分离力的内部判据**（不需要外部参考）——
     `dev`（返程↔去程最近距离）对它**零分离**（最坏轮与干净轮逐位相同，见 CHANGELOG 续 20）。
     缺 traj_twist.txt 时不判该条，但摘要里显式标"未测"，绝不默认通过。
     窗口默认 600–700 s（本数据集塌陷的固定发生段；换数据集须改 --vmin-win）。

用法:
  python3 tools/gate_run.py exp_repeat/a30_r2            # 判读单轮
  python3 tools/gate_run.py exp_repeat/a30_r2 --csv      # 附一行 CSV
  python3 tools/gate_run.py <dir> --span 816.8 --max-rmse 6.0 [--vmin-min 0.10]
"""
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from eval_asymmetry import load_tum                      # noqa: E402

MAX_RMSE = 1.0
MAX_GAP_SEC = 0.5
MIN_COVERAGE = 95.0
MIN_SPAN_RATIO = 0.95
MIN_VMIN = 0.10                       # 判据 ④ 默认阈值 m/s
VMIN_WIN = (600.0, 700.0)             # 判据 ④ 默认窗口（相对本轮起点，s）


def judge(d, expected_span=None, max_rmse=MAX_RMSE, min_vmin=MIN_VMIN, vmin_win=VMIN_WIN):
    d = pathlib.Path(d)
    out = {'dir': str(d), 'valid': False, 'reasons': [],
           'rmse_all': None, 'rmse_out': None, 'rmse_ret': None, 'ratio': None,
           'deg': None, 'coverage': None, 'span': None, 'vmin': None}

    ev = d / 'eval.log'
    if ev.exists():
        txt = ev.read_text(errors='ignore')
        def num(pat, s=txt):
            m = re.search(pat, s)
            return float(m.group(1)) if m else None
        out['rmse_all'] = num(r'全程\s+\d+\s+([\d.]+)')
        out['rmse_out'] = num(r'去程\s+\d+\s+([\d.]+)')
        out['rmse_ret'] = num(r'返程\s+\d+\s+([\d.]+)')
        out['ratio'] = num(r'不对称率\(max\)\s+([\d.]+)')
    else:
        out['reasons'].append('无 eval.log')

    lg = d / 'launch.log'
    if lg.exists():
        out['deg'] = sum(1 for l in lg.open(errors='ignore')
                         if 'suppressed weakest direction' in l)
    else:
        out['reasons'].append('无 launch.log')

    t0 = None
    tr = d / 'traj.txt'
    if tr.exists():
        ts, _, _ = load_tum(str(tr))
        t0 = float(ts[0])
        dt = np.diff(ts)
        span = ts[-1] - ts[0]
        out['span'] = span
        out['coverage'] = 100.0 * (1.0 - dt[dt > MAX_GAP_SEC].sum() / span)
    else:
        out['reasons'].append('无 traj.txt')

    tw = d / 'traj_twist.txt'                      # 判据 ④ 的数据源（export_traj.py 顺带产出）
    if tw.exists() and t0 is not None:
        a = np.loadtxt(str(tw))
        if a.ndim == 1:
            a = a.reshape(1, -1)
        m = (a[:, 0] - t0 >= vmin_win[0]) & (a[:, 0] - t0 <= vmin_win[1])
        if m.sum():
            out['vmin'] = float(np.hypot(a[m, 1], a[m, 2]).min())

    if out['rmse_all'] is None:
        out['reasons'].append('评估无有效数值（eval 失败或记录不完整）')
    elif out['rmse_all'] >= max_rmse:
        out['reasons'].append(f"未收敛 (RMSE={out['rmse_all']:.3g})")
    if out['deg'] is not None and out['deg'] > 0:
        out['reasons'].append(f"初始化期退化抑制 {out['deg']} 次")
    if out['coverage'] is not None and out['coverage'] < MIN_COVERAGE:
        out['reasons'].append(f"覆盖率 {out['coverage']:.1f}%")
    if expected_span and out['span'] is not None and out['span'] < MIN_SPAN_RATIO * expected_span:
        out['reasons'].append(f"记录时长不足 ({out['span']:.0f}s / 期望 {expected_span:.0f}s)")
    if out['vmin'] is not None and out['vmin'] <= min_vmin:
        out['reasons'].append(f"速度塌陷 (vmin={out['vmin']:.3f} m/s ≤ {min_vmin})")

    out['valid'] = not out['reasons']
    return out


def main():
    argv = sys.argv[1:]
    span, max_rmse, min_vmin, vmin_win = None, MAX_RMSE, MIN_VMIN, VMIN_WIN
    for flag, cast in (('--span', float), ('--max-rmse', float), ('--vmin-min', float),
                       ('--vmin-win', None)):
        if flag in argv:
            i = argv.index(flag)
            if flag == '--vmin-win':
                vmin_win = (float(argv[i + 1]), float(argv[i + 2]))
                del argv[i:i + 3]
            else:
                val = cast(argv[i + 1])
                del argv[i:i + 2]
                if flag == '--span':
                    span = val
                elif flag == '--max-rmse':
                    max_rmse = val
                else:
                    min_vmin = val
    pos = [a for a in argv if not a.startswith('--')]
    if not pos:
        print(__doc__)
        sys.exit(1)
    r = judge(pos[0], expected_span=span, max_rmse=max_rmse, min_vmin=min_vmin, vmin_win=vmin_win)
    fmt = lambda v, f: f if v is None else format(v, f)     # noqa: E731
    print(f"{r['dir']}: {'有效 ✓' if r['valid'] else '无效 ✗ ' + '; '.join(r['reasons'])}")
    print(f"   全程 {fmt(r['rmse_all'],'.3f')} | 去程 {fmt(r['rmse_out'],'.3f')} "
          f"| 返程 {fmt(r['rmse_ret'],'.3f')} | 比率 {fmt(r['ratio'],'.3f')}")
    print(f"   退化抑制 {r['deg']} | 覆盖率 {fmt(r['coverage'],'.1f')}% | 跨度 {fmt(r['span'],'.0f')}s")
    print(f"   vmin({vmin_win[0]:.0f}–{vmin_win[1]:.0f}s) "
          f"{fmt(r['vmin'],'.3f')}{'' if r['vmin'] is not None else '（未测）'}"
          f"  ← 干净 0.62–0.69 / 塌陷 ≤0.04（零重叠）")
    if '--csv' in argv:
        cells = []
        for k in ('dir', 'valid', 'rmse_all', 'rmse_out', 'rmse_ret', 'ratio', 'deg', 'coverage', 'vmin'):
            v = r[k]
            if v is None:
                cells.append('')
            elif isinstance(v, str):          # dir 是字符串，不能进 round()
                cells.append(v)
            else:
                cells.append(str(round(v, 4)))
        print(','.join(cells))


if __name__ == '__main__':
    main()
