#!/usr/bin/env python3
"""
误差台阶分析 — 固定先验地图重定位 Calib OFF 的逐帧误差曲线
关键点：reloc vs reloc 同处于先验地图坐标系，直接逐帧差，不做全局 Umeyama 对齐
（对齐会把系统性偏移在去/返两段间分摊，抹平台阶）
输出：1) 台阶统计核对  2) step_curves.png（误差曲线 + yaw 双面板）
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('/home/creepert/Super-LIO/tools')
from eval_asymmetry import load_tum, sync_trajectories, find_turning_point

REF = '/home/creepert/Super-LIO/traj/reloc_traj.txt'
EST = '/home/creepert/Super-LIO/traj/reloc_calib_off.txt'
SPLIT_FILE = '/home/creepert/Super-LIO/traj/reloc_traj_split.txt'
OUT = '/home/creepert/Super-LIO/docs/image/step_curves.png'

# ── 载入与同步 ──
ts_r, xyz_r, quat_r = load_tum(REF)
ts_e, xyz_e, quat_e = load_tum(EST)
with open(SPLIT_FILE) as f:
    split_ts = float(f.read().strip())

xyz_r_s, xyz_e_s, ts_s = sync_trajectories(ts_r, xyz_r, ts_e, xyz_e, max_dt=0.2)
print(f"同步帧数: {len(ts_s)}  (参考 {len(ts_r)}, 估计 {len(ts_e)})")

# ── 直接逐帧平移误差（不对齐）──
errs = np.linalg.norm(xyz_e_s - xyz_r_s, axis=1)
t = ts_s - ts_r[0]                      # 相对秒
t_split = split_ts - ts_r[0]
print(f"掉头时刻: t={t_split:.1f} s")

# 四元数 → yaw（TUM: qx qy qz qw），用估计轨迹全帧（仅作掉头可视化）
if quat_e.shape[1] >= 4:
    qw, qx, qy, qz = quat_e[:, 3], quat_e[:, 0], quat_e[:, 1], quat_e[:, 2]
    yaw_t = ts_e - ts_r[0]
    yaw = np.unwrap(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2)))
else:
    yaw = None

# ── 分段 ──
out_mask = ts_s <= split_ts
ret_mask = ts_s >= split_ts
e_out, e_ret = errs[out_mask], errs[ret_mask]
t_out, t_ret = t[out_mask], t[ret_mask]

def stats(e, name):
    med = np.median(e)
    p5, p95 = np.percentile(e, [5, 95])
    e_trim = e[e < np.percentile(e, 99)]
    print(f"  {name:<6} n={len(e):>5}  中位={med:.4f}  均值={e.mean():.4f}  "
          f"5-95%=[{p5:.4f},{p95:.4f}]  剔除99%后均值={e_trim.mean():.4f}")

print("\n=== 台阶统计 ===")
stats(e_out, "去程")
stats(e_ret, "返程")

# 1) 去程低位平稳: 5-95% 区间宽度
w_out = np.percentile(e_out, 95) - np.percentile(e_out, 5)
print(f"去程 5-95% 区间宽 = {w_out:.4f} m")

# 2) 掉头前后 ±10 s 均值差
win = 10.0
m_pre = errs[(ts_s >= split_ts - win) & (ts_s < split_ts)].mean()
m_post = errs[(ts_s > split_ts) & (ts_s <= split_ts + win)].mean()
print(f"掉头 ±{win}s 均值: 前 {m_pre:.4f} → 后 {m_post:.4f}  跳变 {m_post - m_pre:.4f} m")

# 3) 返程维持高位: 三等分均值
n3 = len(e_ret) // 3
for i, lbl in enumerate(['返程前1/3', '返程中1/3', '返程后1/3']):
    seg = e_ret[i * n3:(i + 1) * n3]
    print(f"  {lbl}: 均值 {seg.mean():.4f}  中位 {np.median(seg):.4f}")

# 4) 台阶 vs 尖峰: 返程中超出"去程上限"的帧占比
thr = np.percentile(e_out, 95)
frac = np.mean(e_ret > thr)
print(f"返程误差 > 去程p95({thr:.4f}) 的帧占比 = {frac*100:.1f}%  "
      f"(台阶型应≫20%, 尖峰型应<5%)")

# 5) 简单变点检测: 在掉头点附近 ±60s 内扫描最佳二分段分割
k_best, var_best = None, np.inf
idx_split = np.searchsorted(ts_s, split_ts)
lo, hi = max(1, idx_split - 600), min(len(ts_s) - 2, idx_split + 600)
for k in range(lo, hi):
    v = errs[:k].var() * k + errs[k:].var() * (len(errs) - k)
    if v < var_best:
        var_best, k_best = v, k
dt_cp = ts_s[k_best] - split_ts
print(f"变点检测: t={ts_s[k_best] - ts_r[0]:.1f} s  与掉头点偏差 {dt_cp:+.1f} s")

# ── 绘图 ──
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True,
                               gridspec_kw={'height_ratios': [2.4, 1], 'hspace': 0.08})

# 去/返底色
ax1.axvspan(t[0], t_split, color='#2980B9', alpha=0.07)
ax1.axvspan(t_split, t[-1], color='#E74C3C', alpha=0.07)
ax2.axvspan(t[0], t_split, color='#2980B9', alpha=0.07)
ax2.axvspan(t_split, t[-1], color='#E74C3C', alpha=0.07)

# 逐帧误差（细线）+ 滑窗中位数（粗线, ~10 s ≈ 90 帧）
ax1.plot(t, errs, color='0.65', lw=0.5, alpha=0.6, label='Per-frame error')
from numpy.lib.stride_tricks import sliding_window_view
w = 91
pad = np.pad(errs, (w // 2, w - 1 - w // 2), mode='edge')
med = np.median(sliding_window_view(pad, w), axis=1)
ax1.plot(t, med, color='#E74C3C', lw=2.0, label='Sliding median (10 s)')

# 两段水平中位线
ax1.axhline(np.median(e_out), color='#2471A3', ls='--', lw=1.2)
ax1.axhline(np.median(e_ret), color='#C0392B', ls='--', lw=1.2)
ax1.text(t[0] + 5, np.median(e_out) + 0.004, f'outbound med {np.median(e_out):.3f} m',
         fontsize=9, color='#2471A3')
ax1.text(t_split + 8, np.median(e_ret) + 0.004, f'return med {np.median(e_ret):.3f} m',
         fontsize=9, color='#C0392B')

ax1.axvline(t_split, color='0.2', ls=':', lw=1.5)
ax1.text(t_split + 3, ax1.get_ylim()[1] * 0.9, '180° turn', fontsize=9, color='0.2')
ax1.set_ylabel('Translation error (m)', fontsize=11)
ax1.set_ylim(bottom=0)
ax1.legend(fontsize=9, loc='upper left')
ax1.grid(True, alpha=0.3)

if yaw is not None:
    ax2.plot(yaw_t, np.degrees(yaw), color='#8E44AD', lw=1.0)
    ax2.set_ylabel('Heading yaw (°)', fontsize=11)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.grid(True, alpha=0.3)
ax2.set_xlim(t[0], t[-1])

ax1.set_title('Per-frame error of relocalization with calibration OFF (fixed prior map, no alignment)',
              fontsize=12, fontweight='bold')
plt.savefig(OUT, dpi=200, bbox_inches='tight')
plt.close()
print(f"\nSaved: {OUT}")
