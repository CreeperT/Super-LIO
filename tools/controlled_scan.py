#!/usr/bin/env python3
"""受控演示 步骤3：沿走廊轴向做偏移扫描。
点云已在**地图系**（见 controlled_step1.py），故 ICP 初值取单位阵+轴向偏移，
不再乘位姿（否则双重变换，点云会被推出 200 m）。"""
import numpy as np, open3d as o3d, sys
SC="/home/creepert/Super-LIO/exp_controlled"
MAP="/home/creepert/Super-LIO/src/super_lio/map/GroutingHole-DrainageHole.pcd"
pose=np.load(f"{SC}/ref_pose.npy"); xyz=pose[:3]

p_scan=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.load(f"{SC}/scan_map.npy")))
d=np.loadtxt("/home/creepert/Super-LIO/traj/reloc_traj.txt")
ref=d[:,1:4]; c=ref-ref.mean(0)
_,_,Vt=np.linalg.svd(c,full_matrices=False); axis=Vt[0]; axis/=np.linalg.norm(axis)
print(f"走廊轴向 (PCA) = {axis.round(4)}")

m=o3d.io.read_point_cloud(MAP); P=np.asarray(m.points)
sel=(np.abs(P[:,0]-xyz[0])<70)&(np.abs(P[:,1]-xyz[1])<70)
tgt=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P[sel])).voxel_down_sample(0.10)
tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.6,max_nn=30))
tgt.orient_normals_towards_camera_location(xyz)
tp=np.asarray(tgt.points); tn=np.asarray(tgt.normals); kd=o3d.geometry.KDTreeFlann(tgt)
print(f"目标地图 {len(tp)} 点（10cm 降采样，法向量朝向传感器）")

def residual_and_H(T):
    ps=np.asarray(p_scan.points); n=len(ps)
    pw=(T[:3,:3]@ps.T).T+T[:3,3]
    res=[];Js=[]
    for i in range(0,n,3):
        k,idx,_=kd.search_knn_vector_3d(pw[i],1)
        if k<1: continue
        j=idx[0]; nv=tn[j]; res.append(abs(nv@(pw[i]-tp[j])))
        pr=ps[i]-T[:3,3]      # 相对传感器位置（绕当前位置旋转，非绕地图原点）
        Js.append(np.concatenate([-nv@np.array([[0,-pr[2],pr[1]],[pr[2],0,-pr[0]],[-pr[1],pr[0],0]]),nv]))
    Js=np.array(Js);res=np.array(res)
    if len(Js)<100: return np.nan,np.nan,np.nan,0
    ev=np.linalg.eigvalsh(Js.T@Js)
    return np.median(res),np.sqrt((res**2).mean()),ev[0],len(Js)

offs=eval(sys.argv[1]) if len(sys.argv)>1 else [0.0]
print(f"\n{'偏移(m)':>8} {'残差中位(cm)':>13} {'残差RMS(cm)':>13} {'λ_min(H)':>12} {'λ_max(H)':>12} {'匹配数':>7}")
rows=[]
for off in offs:
    T=np.eye(4); T[:3,3]=axis*off
    reg=o3d.pipelines.registration.registration_icp(
        p_scan,tgt,2.5,T,o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    rm,rr,lmin,nm=residual_and_H(reg.transformation)
    # 同时给出 λ_max：良态性要看条件数而非单一特征值
    ps=np.asarray(p_scan.points); pw=(reg.transformation[:3,:3]@ps.T).T+reg.transformation[:3,3]
    Js=[]
    for i in range(0,len(ps),3):
        k,idx,_=kd.search_knn_vector_3d(pw[i],1)
        if k<1: continue
        nv=tn[idx[0]]; pr=ps[i]-reg.transformation[:3,3]
        Js.append(np.concatenate([-nv@np.array([[0,-pr[2],pr[1]],[pr[2],0,-pr[0]],[-pr[1],pr[0],0]]),nv]))
    ev=np.linalg.eigvalsh(np.array(Js).T@np.array(Js))
    # ICP 实际位移（沿轴向），确认它是否真的收敛
    dt=reg.transformation[:3,3]; shift=dt@axis
    rows.append((off,rm*100,rr*100,ev[0],ev[-1],nm,shift))
    print(f"{off:8.1f} {rm*100:13.2f} {rr*100:13.2f} {ev[0]:12.4g} {ev[-1]:12.4g} {nm:7d}   ICP轴向位移={shift:+7.3f}m")
np.save(f"{SC}/scan_results.npy",np.array(rows))
