#!/usr/bin/env python3
"""分段对照：把逐帧 ΔC/H_pose 量按"指定路段 vs 其余"分组，回答式(20) 的放大由谁主导。

对给定路段窗口 [t_lo, t_hi]（相对轨迹起点，秒）输出：
  轨迹误差（段内/段外 RMSE、段内对全程 MSE 的贡献）
  λ_min（退化增益 1/λ_min）、‖ΔC‖、‖ΔCξ_Δ‖、弱方向投影 v_minᵀΔCξ_Δ、预测偏移 ‖H⁻¹ΔCξ_Δ‖
其中"弱方向偏移分量" = 投影/λ_min，其段内外比值可分解为
  (投影比值) × (1/λ_min 比值) —— 用于判断"退化增益"与"方向对准"各自贡献多少。

用法：
  python3 segment_projection_report.py --traj exp_pose_dc/traj_a30.txt \
      --ref traj/reloc_traj.txt --csv exp_pose_dc/pose_dc_a30.csv --seg 765 790 \
      --xi-deg 0,0.7,0,0,0,0 --plot exp_pose_dc/a30_segment.png
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, '/home/creepert/Super-LIO/tools')
from eval_asymmetry import load_tum                      # noqa: E402
from analyze_pose_dc import load_csv, per_frame, xi_from_deg  # noqa: E402


def interp_ref_on_est(ts_e, xyz_e, ts_r, xyz_r):
    """把参考轨迹插值到估计帧时间戳上（保持估计帧索引，便于与 CSV 行对齐）。"""
    idx = np.clip(np.searchsorted(ts_r, ts_e), 1, len(ts_r) - 1)
    t0, t1 = ts_r[idx - 1], ts_r[idx]
    w = np.where(t1 > t0, (ts_e - t0) / np.maximum(t1 - t0, 1e-9), 0.0)[:, None]
    return xyz_r[idx - 1] * (1 - w) + xyz_r[idx] * w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', required=True, help='估计轨迹 (export_traj 输出)')
    ap.add_argument('--ref', required=True, help='参考轨迹')
    ap.add_argument('--csv', required=True, help='pose_dc_v2.csv')
    ap.add_argument('--seg', nargs=2, type=float, required=True, metavar=('T_LO', 'T_HI'),
                    help='路段窗口（相对轨迹起点，秒）')
    ap.add_argument('--xi-deg', default='0,0.7,0,0,0,0')
    ap.add_argument('--trim', type=float, default=30.0, help='剔除起始 N 秒（默认 30）')
    ap.add_argument('--plot', default=None, help='输出图路径')
    args = ap.parse_args()

    ts_e, xyz_e, _ = load_tum(args.traj)
    ts_r, xyz_r, _ = load_tum(args.ref)
    xyz_r_i = interp_ref_on_est(ts_e, xyz_e, ts_r, xyz_r)
    err = np.linalg.norm(xyz_e - xyz_r_i, axis=1)
    t = ts_e - ts_e[0]

    frame, _, H, D1, D2, _ = load_csv(args.csv)
    res = per_frame(H, D1 - D2, xi_from_deg(args.xi_deg))

    n = min(len(err), len(frame))
    err, t = err[:n], t[:n]
    seg = (t >= args.seg[0]) & (t <= args.seg[1])
    keep = t >= args.trim                      # 与 §4.1 同口径：剔除前 30 s
    seg &= keep

    rmse = lambda m: float(np.sqrt((err[m] ** 2).mean()))
    print(f'路段 [{args.seg[0]:.0f}, {args.seg[1]:.0f}] s：占 {100 * seg.mean():.1f}% 帧'
          f'（已剔除前 {args.trim:.0f} s）')
    print(f'  轨迹 RMSE：段内 {rmse(seg):.3f} m | 段外 {rmse(keep & ~seg):.3f} m'
          f' | 段内 MSE 占 {100 * (err[seg] ** 2).sum() / (err[keep] ** 2).sum():.1f}%')

    rows = [('λ_min', 'lam_min'), ('‖ΔC‖', 'sv1'), ('‖ΔCξ_Δ‖', 'force'),
            ('弱方向投影', 'proj'), ('预测偏移', 'pred')]
    med = {}
    print(f'  {"量":<12}{"段内中位":>13}{"段外中位":>13}{"比值":>8}')
    for name, key in rows:
        v = res[key][:n]
        a, b = float(np.median(v[seg])), float(np.median(v[keep & ~seg]))
        med[name] = (a, b)
        print(f'  {name:<12}{a:>13.3e}{b:>13.3e}{a / (b + 1e-30):>8.2f}')
    gain = med['λ_min'][1] / (med['λ_min'][0] + 1e-30)          # 1/λ_min 的放大倍数
    proj = abs(med['弱方向投影'][0]) / (abs(med['弱方向投影'][1]) + 1e-30)
    print(f'\n  弱方向偏移分量 = 投影/λ_min：段内放大 {proj * gain:.1f}×'
          f'（其中退化增益 {gain:.1f}× 、方向对准 {proj:.1f}×）')
    print(f'  ⇒ 主导项：{"方向对准（ΔC 结构）" if proj > gain else "退化增益（1/λ_min）"}'
          f'，两者相差 {max(proj, gain) / max(min(proj, gain), 1e-30):.1f}×')

    ratio = np.abs(res['proj'][:n]) / np.maximum(res['force'][:n], 1e-30)
    print(f'  投影/力：段内中位 {np.median(ratio[seg]):.4f} | 段外中位 {np.median(ratio[keep & ~seg]):.4f}')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.family": "serif", "font.size": 7, "axes.linewidth": .6})
        INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e1e0d9"
        tm = t / 60.0                                    # 与图 9 一致：横轴用分钟
        slo, shi = args.seg[0] / 60.0, args.seg[1] / 60.0
        fig, ax = plt.subplots(4, 1, figsize=(3.45, 5.2),
                               gridspec_kw={"hspace": .42, "height_ratios": [1, 1, 1, 1.2]})
        rows = [(err, '#E74C3C', 'ATE [m]', '轨迹误差'),
                (res['lam_min'][:n], '#2980B9', r'$\lambda_{\min}(\mathbf{H}_{\mathrm{pose}})$', 'λ_min'),
                (np.abs(res['proj'][:n]), '#27AE60',
                 r'$|\mathbf{v}_{\min}^{\top}\Delta\mathbf{C}\,\xi_\Delta|$', '弱方向投影')]
        for a_, (v, c, ylab, key) in zip(ax[:3], rows):
            a_.plot(tm, v, color=c, lw=.7)
            a_.set_yscale('log'); a_.set_ylabel(ylab, fontsize=7)
            a_.axvspan(slo, shi, color=GRID, alpha=.9, zorder=0)
            a_.axvline(slo, color=MUTED, ls='--', lw=.6)
            a_.axvline(shi, color=MUTED, ls='--', lw=.6)
            if key in med:
                r_ = med[key][0] / (med[key][1] + 1e-30)
                a_.annotate('segment / rest = %.2f$\\times$' % r_, xy=(.98, .06),
                            xycoords='axes fraction', ha='right', fontsize=6.5, color=INK)
            a_.grid(color=GRID, lw=.4, alpha=.8); a_.set_axisbelow(True)
            for sp in ('top', 'right'):
                a_.spines[sp].set_visible(False)
        ax[2].set_xlabel('time [min]', fontsize=7)
        ax[0].annotate('weak-constraint segment', xy=(slo - .25, err.max() * .5), fontsize=6.5,
                       color=MUTED, ha='right')

        # ── 末行: 该段的放大条带(三个量各自按"段外中位"归一化, 1.0 = 段外水平) ──
        a_ = ax[3]
        zs = [('ATE', err, '#E74C3C'), (r'$\lambda_{\min}$', res['lam_min'][:n], '#2980B9'),
              (r'$|\mathbf{v}_{\top}\Delta\mathbf{C}\xi_\Delta|$', np.abs(res['proj'][:n]), '#27AE60')]
        for lbl, v, c in zs:
            base = np.median(v[keep & ~seg])
            a_.plot(t, v / (base + 1e-30), color=c, lw=.7, label=lbl)
        a_.set_yscale('log'); a_.set_xlim(args.seg[0] - 12, args.seg[1] + 12)
        a_.set_ylabel('segment / rest', fontsize=7); a_.set_xlabel('time [s]', fontsize=7)
        a_.axvspan(args.seg[0], args.seg[1], color=GRID, alpha=.9, zorder=0)
        a_.axhline(1.0, color=MUTED, ls=':', lw=.6)
        lo, hi = a_.get_ylim()
        a_.set_ylim(lo, hi * 4.0)            # 顶部留白，图例置于其中而不压曲线
        a_.legend(fontsize=6.5, frameon=False, ncol=3, loc='upper center')
        a_.grid(color=GRID, lw=.4, alpha=.8); a_.set_axisbelow(True)
        for sp in ('top', 'right'):
            a_.spines[sp].set_visible(False)
        fig.tight_layout()
        fig.subplots_adjust(left=.20, right=.985, top=.98, bottom=.06)   # 防左侧标签被裁
        fig.savefig(args.plot, dpi=600)
        print(f'  图已保存: {args.plot}')


if __name__ == '__main__':
    main()
