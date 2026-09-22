#!/usr/bin/env python3
"""式(20) 定量验证：由逐帧 H_pose 与 ΔC 计算退化增益、失配投影与预测偏移。

数据来源（SuperLIO::Observe() 导出，两版格式自动识别）：
  logs/pose_dc.csv     列: frame, H_pose(36), ΔC1(36), ΔC2(36)                             —— 109 列
  logs/pose_dc_v2.csv  列: frame, t, H_pose(36), ΔC1(36), ΔC2(36), rel_rpy(3), rel_xyz(3) —— 116 列
H_pose = Σ w·J_poseᵀJ_pose（6×6，行主序，参数序 [旋转(3); 平移(3)]），
ΔC_k = Σ w·J_poseᵀJ_ext,k（6×6）；DC1 对应前向雷达 L1、DC2 对应后向雷达 L2，故 ΔC = DC1 − DC2。

式(20)：δξ*_shift = H_pose⁻¹·ΔC·ξ_Δ，按 H 的特征分解展开，第 i 个模态贡献为 (v_iᵀΔCξ_Δ)/λ_i，
即退化（λ_min→0）只放大 ΔCξ_Δ 在 v_min 方向上的投影。脚本输出 λ_min、‖ΔC‖、
‖ΔCξ_Δ‖、投影 |v_minᵀΔCξ_Δ| 与预测偏移 ‖H⁻¹ΔCξ_Δ‖。

用法：
  python3 analyze_pose_dc.py                            # 汇总所有 run
  python3 analyze_pose_dc.py --run 8 --plot             # 单 run 逐帧曲线
  python3 analyze_pose_dc.py --xi-deg "0,0.7,0,0,0,0"   # 指定 ξ_Δ（旋转以度计）
  python3 analyze_pose_dc.py --xi-from-rel --rel-ref-deg "0,-30,0,-0.16,0,0.09"
"""
import argparse
import os
import sys

import numpy as np

DEFAULT_V2 = '/home/creepert/Super-LIO/src/super_lio/logs/pose_dc_v2.csv'
DEFAULT_V1 = '/home/creepert/Super-LIO/src/super_lio/logs/pose_dc.csv'


# ─────────────────────────── 数据读取 ───────────────────────────

def load_csv(path):
    """返回 (frame, time|None, H_pose(N,6,6), DC1(N,6,6), DC2(N,6,6), rel(N,6)|None)。

    逐行解析并跳过不完整行（进程被 kill 时最后一行可能写了一半）。"""
    rows = []
    ncol = None
    for line in open(path):
        v = line.rstrip('\n').split(',')
        if ncol is None:
            if len(v) not in (109, 116):
                continue
            ncol = len(v)
        if len(v) == ncol:
            rows.append([float(x) for x in v])
    if not rows:
        sys.exit(f'未解析到有效数据行：{path}')
    d = np.array(rows)
    ncol = d.shape[1]
    if ncol == 109:      # v1: frame + 3×36
        frame, t, off, rel = d[:, 0], None, 1, None
    elif ncol == 116:    # v2: frame + time + 3×36 + rel(6)
        frame, t, off = d[:, 0], d[:, 1], 2
        rel = d[:, off + 108:off + 114]
    else:
        sys.exit(f'未知列数 {ncol}（应为 109 或 116）：{path}')
    H = d[:, off:off + 36].reshape(-1, 6, 6)
    DC1 = d[:, off + 36:off + 72].reshape(-1, 6, 6)
    DC2 = d[:, off + 72:off + 108].reshape(-1, 6, 6)
    return frame, t, H, DC1, DC2, rel


def split_runs(frame):
    """帧号回退即视为新 run（CSV 为 append 模式，含多次运行）。"""
    brk = [0] + [i for i in range(1, len(frame)) if frame[i] <= frame[i - 1]] + [len(frame)]
    return [(brk[k], brk[k + 1]) for k in range(len(brk) - 1)]


# ─────────────────────────── 核心计算 ───────────────────────────

def xi_from_deg(s):
    """'0,0.7,0,0,0,0' → ξ = [θ(3) rad; ρ(3)]，其中角度以度输入。"""
    v = np.array([float(x) for x in s.split(',')], dtype=float)
    if v.size != 6:
        sys.exit('--xi-deg 需 6 个分量：θx,θy,θz(度), ρx,ρy,ρz')
    v[:3] = np.deg2rad(v[:3])
    return v


def xi_from_relative(rel, rel_ref):
    """由逐帧相对外参 T_L1⁻¹T_L2 与参考值之差构造 ξ_Δ（小角近似的右扰动）。"""
    xi = np.zeros_like(rel)
    xi[:, :3] = rel_ref[:3] - rel[:, :3]
    xi[:, 3:] = rel[:, 3:] - rel_ref[3:]
    return xi


def per_frame(H, DC, xi):
    """逐帧计算 λ_min、‖ΔC‖、‖ΔCξ_Δ‖、弱方向投影与预测偏移。

    预测偏移按式(20) 的特征分解逐模态求和：δ = Σ_i (v_iᵀΔCξ_Δ)/λ_i · v_i。
    不可观方向（λ_i ≈ 0，即空观测帧或完全退化帧）不计入，另记为 n_sing；
    同时给出上界形式 ‖ΔCξ_Δ‖/λ_min（式(23)），该值在退化时发散。xi 可为 (6,) 或 (N,6)。
    """
    n = len(H)
    lam_min = np.empty(n); proj = np.empty(n)
    force = np.empty(n); pred = np.empty(n); sv1 = np.empty(n)
    bound = np.empty(n); n_sing = np.zeros(n, dtype=int)
    for i in range(n):
        Hm = H[i]
        lam, vec = np.linalg.eigh(Hm)              # 对称矩阵，升序
        f = DC[i] @ (xi if xi.ndim == 1 else xi[i])
        tol = max(1e-12 * lam[-1], 1e-12)          # 相对阈值，避免把空观测帧当无穷放大
        ok = lam > tol
        coef = vec.T @ f
        delta = vec[:, ok] @ (coef[ok] / lam[ok]) if ok.any() else np.zeros(6)
        lam_min[i] = lam[0]
        force[i] = np.linalg.norm(f)
        proj[i] = vec[:, 0] @ f                    # ΔCξ_Δ 在弱特征方向上的投影
        pred[i] = np.linalg.norm(delta)
        bound[i] = np.linalg.norm(f) / lam[0] if lam[0] > tol else np.inf
        n_sing[i] = int((~ok).sum())
        sv1[i] = np.linalg.svd(DC[i], compute_uv=False)[0]
    return dict(lam_min=lam_min, force=force, proj=proj, pred=pred, sv1=sv1,
                bound=bound, n_sing=n_sing)


def q(a, p=50):
    return float(np.percentile(a, p))


# ─────────────────────────── 输出 ───────────────────────────

def summarize(frame, t, runs, res):
    hdr = (f'{"run":>4} {"n":>5} {"帧范围":>14} {"时长(s)":>8} {"ΔC":>3} {"λmin中位":>10} '
           f'{"λminP5":>10} {"‖ΔCξ‖中位":>11} {"投影中位":>11} {"|投影|P95":>11} '
           f'{"预测偏移中位":>12}')
    print(hdr)
    print('-' * len(hdr))
    for k, (a, b) in enumerate(runs):
        if b - a < 50:
            continue
        r = res[k]
        dur = f'{(t[b - 1] - t[a]):.1f}' if t is not None else '-'
        has_dc = '有' if np.median(r['sv1']) > 0 else '无'
        print(f'{k:>4} {b - a:>5} {int(frame[a]):>6}→{int(frame[b - 1]):<6} {dur:>8} {has_dc:>3} '
              f'{np.median(r["lam_min"]):>10.3e} {q(r["lam_min"], 5):>10.3e} '
              f'{np.median(r["force"]):>11.3e} {np.median(r["proj"]):>11.3e} '
              f'{q(np.abs(r["proj"]), 95):>11.3e} {np.median(r["pred"]):>12.3e}')


def plot_run(k, frame, t, runs, res, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    a, b = runs[k]
    x = (t[a:b] - t[a]) if t is not None else frame[a:b]
    xlabel = 'Time (s)' if t is not None else 'Frame'
    r = res[k]
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    ax[0].plot(x, r['lam_min'], 'C3', lw=1.0); ax[0].set_yscale('log')
    ax[0].set_ylabel(r'$\lambda_{\min}(\mathbf{H}_{pose})$')
    ax[1].plot(x, r['sv1'], 'C0', lw=1.0); ax[1].set_yscale('log')
    ax[1].set_ylabel(r'$\|\Delta\mathbf{C}\|$ ($\sigma_1$)')
    ax[2].plot(x, np.abs(r['proj']), 'C2', lw=1.0); ax[2].set_yscale('log')
    ax[2].set_ylabel(r'$|\mathbf{v}_{\min}^\top \Delta\mathbf{C}\,\xi_\Delta|$')
    ax[3].plot(x, r['pred'], 'C4', lw=1.0); ax[3].set_yscale('log')
    ax[3].set_ylabel(r'$\|\mathbf{H}^{-1}\Delta\mathbf{C}\,\xi_\Delta\|$')
    ax[3].set_xlabel(xlabel)
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.suptitle(f'run {k}  ({int(frame[a])}→{int(frame[b - 1])})')
    fig.tight_layout()
    path = os.path.join(outdir, f'pose_dc_run{k}.png')
    fig.savefig(path, dpi=150)
    print(f'图已保存: {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=None, help='pose_dc_v2.csv / pose_dc.csv')
    ap.add_argument('--run', type=int, default=None, help='只分析该 run')
    ap.add_argument('--xi-deg', default='0,0.7,0,0,0,0',
                    help='ξ_Δ：θx,θy,θz(度),ρx,ρy,ρz（默认 0.7° pitch）')
    ap.add_argument('--xi-from-rel', action='store_true',
                    help='由逐帧相对外参减参考值推 ξ_Δ（需 v2 格式）')
    ap.add_argument('--rel-ref-deg', default=None,
                    help='参考相对外参 θx,θy,θz(度),tx,ty,tz')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--outdir', default='/home/creepert/Super-LIO/docs/image')
    args = ap.parse_args()

    path = args.csv or (DEFAULT_V2 if os.path.exists(DEFAULT_V2) else DEFAULT_V1)
    print(f'读取: {path}')
    frame, t, H, DC1, DC2, rel = load_csv(path)
    DC = DC1 - DC2
    runs = split_runs(frame)
    print(f'共 {len(runs)} 段 run，{len(frame)} 帧\n')

    if args.xi_from_rel:
        if rel is None:
            sys.exit('--xi-from-rel 需要 v2 格式（含相对外参列）')
        if args.rel_ref_deg is None:
            sys.exit('--xi-from-rel 需同时给 --rel-ref-deg')
        ref = np.array([float(x) for x in args.rel_ref_deg.split(',')])
        ref[:3] = np.deg2rad(ref[:3])
        xi = xi_from_relative(rel, ref)
        label = f'由相对外参推得（参考 {args.rel_ref_deg}）'
    else:
        xi = xi_from_deg(args.xi_deg)
        label = args.xi_deg + '（θ 以度计）'
    print(f'ξ_Δ = {label}\n')

    res = {k: per_frame(H[a:b], DC[a:b], xi) for k, (a, b) in enumerate(runs)}

    if args.run is not None:
        a, b = runs[args.run]
        r = res[args.run]
        print(f'run {args.run}: 帧 {int(frame[a])}→{int(frame[b - 1])}')
        for key, name in [('lam_min', 'λmin'), ('sv1', '‖ΔC‖'), ('force', '‖ΔCξ_Δ‖'),
                          ('proj', '弱方向投影'), ('pred', '预测偏移')]:
            v = r[key]
            print(f'  {name:<10} 中位 {np.median(v):.3e}  P5 {q(v, 5):.3e}  P95 {q(v, 95):.3e}')
        if args.plot:
            plot_run(args.run, frame, t, runs, res, args.outdir)
    else:
        summarize(frame, t, runs, res)
        if args.plot:
            for k, (a, b) in enumerate(runs):
                if b - a >= 500:
                    plot_run(k, frame, t, runs, res, args.outdir)


if __name__ == '__main__':
    main()
