#!/usr/bin/env python3
"""δt 归属分析：对比 pitch5 注入组（bpitch5t）与无注入组（bcalibon）的 δt 终值。
若两组 δt2 同量级 → δt2 是时基残差（归属成立）；
若 bcalibon 显著更小 → δt2 含旋转误差耦合项（表 VIII 该行解释须重写）。"""
import numpy as np
rows=[]
for ln in open('/home/creepert/Super-LIO/src/super_lio/logs/solver_time.csv',errors='ignore'):
    p=ln.strip().split(',')
    if len(p)!=6: continue
    try: rows.append([float(x) for x in p])
    except: pass
d=np.array(rows); f=d[:,0]; runs=np.split(d,np.where(np.diff(f)<0)[0]+1)
print("=== 全部完整段（>1000 帧）===")
print(f"{'段':>3} {'帧数':>6} {'δt1终值(ms)':>12} {'δt2终值(ms)':>12} {'δt1收敛帧':>10} {'δt2收敛帧':>10}")
full=[]
for i,r in enumerate(runs):
    n=len(r)
    if n<1000: continue
    dt1,dt2=r[:,4]*1000,r[:,5]*1000
    if np.all(np.abs(dt1)<1e-9) and np.all(np.abs(dt2)<1e-9): continue
    def conv(v):
        tgt=np.median(v[-1000:]); band=np.abs(v-tgt)<=0.5; c=0
        for k,b in enumerate(band):
            c=c+1 if b else 0
            if c>=300: return k-299
        return None
    full.append((i+1,n,float(np.median(dt1[-1000:])),float(np.median(dt2[-1000:])),conv(dt1),conv(dt2)))
    print(f"{i+1:>3} {n:>6} {full[-1][2]:>12.3f} {full[-1][3]:>12.3f} {str(full[-1][4]):>10} {str(full[-1][5]):>10}")
if len(full)>=4:
    inj=np.array([x[3] for x in full[:3]]); noi=np.array([x[3] for x in full[3:]])
    print(f"\n★ 归属对比（δt2）：")
    print(f"  pitch5 注入组 (n={len(inj)}): {inj.mean():.3f} ± {inj.std():.3f} ms   {inj.round(3)}")
    print(f"  无注入组     (n={len(noi)}): {noi.mean():.3f} ± {noi.std():.3f} ms   {noi.round(3)}")
    diff=abs(inj.mean()-noi.mean()); pooled=np.sqrt(inj.var()+noi.var())/np.sqrt(2)
    print(f"  两组均值差 = {diff:.3f} ms（组内合并标准差 {pooled:.3f} ms）")
    if diff < 3*pooled:
        print(f"  → 差异在组内离散度内（<3σ）：**δt2 与旋转注入无关，归属成立**——它是时基残差")
    else:
        print(f"  → 差异显著（>{3}σ）：**δt2 含旋转耦合项**，表 VIII 该行解释须重写，不可表述为'时基残差'")
