#!/usr/bin/env python3
"""受控演示 步骤4：画轴向偏移扫描的两条曲线。"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
SC="/home/creepert/Super-LIO/exp_controlled"
r=np.load(f"{SC}/scan_results.npy")     # off, res_med(cm), res_rms(cm), lmin, lmax, n, icp_shift
off, rmed, lmin = r[:,0], r[:,1], r[:,3]
ok = rmed < 100                          # 残差 >100cm 视为 ICP 失败，单列

fig,ax=plt.subplots(2,1,figsize=(9,7),sharex=True,gridspec_kw={'height_ratios':[1,1]})
# 上：残差
ax[0].semilogy(off[rmed<100], rmed[rmed<100], 'o-', color='#1f4e79', lw=1.8, ms=6, label='median residual')
ax[0].semilogy(off[rmed>=100], rmed[rmed>=100], 'x', color='#c00000', ms=10, mew=2.5, label='ICP failed (>1 m)')
ax[0].axhline(0.83, color='gray', ls=':', lw=1.2, label='residual at true pose (0.83 cm)')
ax[0].axvspan(-10,5, color='#2e7d32', alpha=0.10)
ax[0].set_ylabel('point-to-plane residual (cm, log)')
ax[0].set_title('Axial offset scan along the corridor (A-segment scan at t=636 s)')
ax[0].legend(fontsize=8,loc='lower right'); ax[0].grid(alpha=0.3,which='both')
# 下：λ_min
ax[1].plot(off[rmed<100], lmin[rmed<100], 's-', color='#b35c00', lw=1.8, ms=6, label=r'$\lambda_{\min}(\mathbf{H})$')
ax[1].plot(off[rmed>=100], lmin[rmed>=100], 'x', color='#c00000', ms=10, mew=2.5)
ax[1].axvspan(-10,5, color='#2e7d32', alpha=0.10, label='flat valley (15 m wide)')
ax[1].set_xlabel('injected axial offset (m)'); ax[1].set_ylabel(r'$\lambda_{\min}(\mathbf{H})$')
ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
ax[1].annotate('residual ×15 worse,\nyet H appears *better*', xy=(7,0.442), xytext=(2,0.33),
               fontsize=8, color='#c00000',
               arrowprops=dict(arrowstyle='->',color='#c00000',lw=1.2))
plt.tight_layout(); plt.savefig(f"{SC}/controlled_scan.png",dpi=160)
print(f"已存 {SC}/controlled_scan.png")
# 数值摘要
v=rmed[(off>=-10)&(off<=5)]
print(f"\n谷底区(−10…+5 m)：残差 {v.min():.2f}–{v.max():.2f} cm | λ_min 变化 "
      f"{lmin[(off>=-10)&(off<=5)].min():.4f}–{lmin[(off>=-10)&(off<=5)].max():.4f} "
      f"(相对变化 {(lmin[(off>=-10)&(off<=5)].max()/lmin[(off>=-10)&(off<=5)].min()-1)*100:.1f}%)")
print(f"谷底外(+6…+8 m)：残差 {rmed[(off>=6)&(off<=8)].mean():.1f} cm | λ_min {lmin[(off>=6)&(off<=8)].mean():.4f}（升高 {(lmin[(off>=6)&(off<=8)].mean()/lmin[off==0][0]-1)*100:.0f}%）")
