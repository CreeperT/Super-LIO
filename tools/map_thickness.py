#!/usr/bin/env python3
"""地图"薄厚"离线自检（判据卡第 1 层之 1.3）：同一体素内重复覆盖点云的**法向散布**。

它测什么、不测什么（重要，见 CHANGELOG 续 20/21）:
  · 对**坡度误差基本免疫** —— 坡度是旋转型误差，图内形状量对它不变（续 20 §1：信息被 2δ×力臂
    压缩到 mm 级，而体素/粗糙度在 cm–m 级）⇒ **别用它判坡度**。
  · 对**非刚性**错误敏感 —— 位姿停滞/速度塌陷会让墙**沿程复制**（米级重影）；沿程漂移/分岔让
    同一面墙在两趟落在不同位置（两腿不一致）⇒ 它是"图是**刚性地**歪，还是**烂了**"的检验。

三种输入:
  1. PCD（`open3d` 读 `binary_compressed`；无 open3d 时自解析未压缩 binary，或用
     `pcl_convert_pcd_ascii_binary in.pcd out.pcd 1` 先转换）
  2. txt（每行 `t x y z`）
  3. **bag + topic**（默认 `/lio/cloud_world`）—— 带时间戳 ⇒ 可**按折返点分腿**，额外给出
     **跨腿质心错位**（同体素内两腿质心之差）：这是"两腿不一致 / 沿程重影"的直接指纹，
     也可读出**腿间旋转的上界**（错位 ÷ 力臂）。

口径:
  · 体素边长 --vox（默认 0.5 m，与 hash_map.vox_resolution 同量级）
  · 每体素最少点数 --minpts（默认 20）；不足则跳过（否则"厚度"只是采样噪声）
  · 厚度 = 该体素点云协方差**最小特征值**的平方根（沿法向的 1σ 散布），单位 m
  · 按最小特征向量分类：|n_z| > cos(--ang) → ground，|n_z| < sin(--ang) → wall，其余 mid（--ang 默认 20°，
    与 lio.structure.max_angle_deg 同口径）
  · 分辨力下限 ≈ --vox：**若地图是体素降采样过的（if_filter: true / ds_size 0.5），低于 0.5 m 的
    重影不可分辨** ⇒ 用未降采样的 PCD，或逐帧点云（多帧采样给同一体素带来多样本）

判据卡 1.3 阈值（先验图实测基准）: 中位 ≤0.05 m、P95 ≤0.15 m、>0.2 m ≤2%
（先验图 6 384 021 点、vox 0.5 ⇒ 0.018 / 0.077 / 0.0%）

用法:
  python3 tools/map_thickness.py src/super_lio/map/GroutingHole-DrainageHole.pcd
  python3 tools/map_thickness.py exp_repeat/x/clouds.txt --vox 0.5 --minpts 20
  python3 tools/map_thickness.py --bag exp_repeat/aslam_stage1_r1/bag --topic /lio/cloud_world
"""
import argparse
import math
import pathlib
import sys

import numpy as np

ANG_DEG = 20.0          # 与 lio.structure.max_angle_deg 同口径
SPLIT_DEFAULT = 1782872767.704156      # 5#GJD 折返点（bag 时间轴）


def load_pcd(path):
    """(N,3) float64。优先 open3d（支持 binary_compressed），否则自解析未压缩 binary。"""
    p = pathlib.Path(path)
    if p.suffix.lower() == '.txt':
        a = np.loadtxt(str(p))
        if a.ndim == 1:
            a = a.reshape(1, -1)
        return a[:, -3:].astype(np.float64)

    try:
        import open3d as o3d
        pts = np.asarray(o3d.io.read_point_cloud(str(p)).points, dtype=np.float64)
        if pts.size:
            return pts
        print('（open3d 读出 0 点，改自解析）')
    except Exception as e:                                    # noqa: BLE001
        print(f'（open3d 不可用：{e}；改自解析）')

    with p.open('rb') as f:
        hdr = {}
        while True:
            raw = f.readline()
            if not raw:
                sys.exit('PCD 头不完整')
            line = raw.decode('ascii', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            k, _, v = line.partition(' ')
            hdr[k.upper()] = v.strip()
            if k.upper() == 'DATA':
                kind = v.strip().lower()
                break
        if kind != 'binary':
            sys.exit(f'自解析只支持 DATA binary（本文件为 {kind}）⇒ 装 open3d 或先 '
                     f'pcl_convert_pcd_ascii_binary in.pcd out.pcd 1')
        fields = hdr['FIELDS'].split()
        sizes = [int(s) for s in hdr['SIZE'].split()]
        types = hdr['TYPE'].split()
        counts = [int(c) for c in hdr.get('COUNT', ' '.join('1' * len(fields))).split()]
        n = int(hdr['POINTS'])
        for req in ('x', 'y', 'z'):
            if req not in fields:
                sys.exit(f'PCD 缺字段 {req}')
        dt = np.dtype([(fields[i], {'F': '<f4', 'U': '<u4', 'I': '<i4'}[types[i]],
                        (counts[i],)) for i in range(len(fields))])
        buf = np.frombuffer(f.read(dt.itemsize * n), dtype=dt, count=n)
        return np.column_stack([buf['x'].astype(np.float64), buf['y'].astype(np.float64),
                                buf['z'].astype(np.float64)])


def _xyz_from_pc2(msg):
    """从 sensor_msgs/PointCloud2 取出 (N,3) float32（不做任何变换）。"""
    off = {fd.name: fd.offset for fd in msg.fields}
    if not all(k in off for k in ('x', 'y', 'z')):
        return None
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return None
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8,
                        count=msg.row_step * msg.height)
    a = raw.reshape(msg.height, msg.row_step)[:, :msg.width * msg.point_step]
    a = a.reshape(-1, msg.point_step)
    cols = []
    for k in ('x', 'y', 'z'):
        o = off[k]
        cols.append(a[:, o:o + 4].copy().view('<f4').ravel())
    P = np.column_stack(cols).astype(np.float32)
    return P[np.isfinite(P).all(axis=1)]


def load_bag_clouds(bagdir, topic, max_clouds=0):
    """读 bag 的 PointCloud2 话题；返回 (时间数组, 点数组列表)。需 ROS 环境。"""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bagdir), storage_id='sqlite3'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=[topic]))
    ts, clouds = [], []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, PointCloud2)
        P = _xyz_from_pc2(m)
        if P is None or len(P) == 0:
            continue
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        clouds.append(P)
        if max_clouds and len(clouds) >= max_clouds:
            break
    return np.array(ts), clouds


def voxel_stats(P, vox, minpts):
    """返回 (厚度, |n_z|, 体素 key 的逆映射, 各体素质心, 各体素计数)。"""
    key = np.floor(P / vox).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    M = len(uniq)
    cnt = np.bincount(inv, minlength=M)
    w = lambda f: np.bincount(inv, weights=f, minlength=M)           # noqa: E731
    mu = np.column_stack([w(P[:, k].astype(np.float64)) / np.maximum(cnt, 1) for k in range(3)])
    C = np.empty((M, 3, 3), dtype=np.float32)
    for a in range(3):
        for b in range(a, 3):
            cab = (w(P[:, a] * P[:, b]) / np.maximum(cnt, 1) - mu[:, a] * mu[:, b]).astype(np.float32)
            C[:, a, b] = cab
            C[:, b, a] = cab
    ok = cnt >= minpts
    ev, evec = np.linalg.eigh(C[ok])
    return np.sqrt(np.clip(ev[:, 0], 0.0, None)), np.abs(evec[:, 2, 0]), inv, mu, cnt, uniq


def report(name, th, nz, vox, minpts, npts):
    cos_lim, sin_lim = math.cos(math.radians(ANG_DEG)), math.sin(math.radians(ANG_DEG))
    cls = np.where(nz > cos_lim, 1, np.where(nz < sin_lim, 2, 0))     # 1=ground 2=wall 0=mid
    print('{}: {} 点 | vox {} m | minpts {} | 合格体素 {}'.format(name, npts, vox, minpts, len(th)))
    print('  {:<8}{:>9}{:>10}{:>9}{:>9}{:>9}'.format('类别', '体素', '厚度中位', 'P95', '>0.2m', '>1.0m'))
    for k, lab in ((1, 'ground'), (2, 'wall'), (0, 'mid')):
        m = cls == k
        if not m.any():
            continue
        t = th[m]
        print('  {:<8}{:>9}{:>10.3f}{:>9.3f}{:>8.1f}%{:>8.1f}%'.format(
            lab, int(m.sum()), float(np.median(t)), float(np.percentile(t, 95)),
            100 * float(np.mean(t > 0.2)), 100 * float(np.mean(t > 1.0))))
    print('  {:<8}{:>9}{:>10.3f}{:>9.3f}{:>8.1f}%{:>8.1f}%'.format(
        '全部', len(th), float(np.median(th)), float(np.percentile(th, 95)),
        100 * float(np.mean(th > 0.2)), 100 * float(np.mean(th > 1.0))))


def cross_leg(uniq_a, mu_a, cnt_a, uniq_b, mu_b, cnt_b, minpts):
    """同体素内、两腿各自的质心之差（仅统计两腿都够点的体素）。"""
    ka = {tuple(r): i for i, r in enumerate(uniq_a)}
    idx_b, idx_a = [], []
    for i, r in enumerate(uniq_b):
        j = ka.get(tuple(r))
        if j is not None and cnt_a[j] >= minpts and cnt_b[i] >= minpts:
            idx_b.append(i)
            idx_a.append(j)
    if not idx_a:
        return None
    d = mu_b[idx_b] - mu_a[idx_a]
    dist = np.linalg.norm(d, axis=1)
    return {'n': len(idx_a), 'dist_med': float(np.median(dist)),
            'dist_p95': float(np.percentile(dist, 95)), 'dist_max': float(dist.max()),
            'dx_med': float(np.median(d[:, 0])), 'dy_med': float(np.median(d[:, 1])),
            'dz_med': float(np.median(d[:, 2])),
            'dz_p95': float(np.percentile(np.abs(d[:, 2]), 95))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd', nargs='*', help='PCD / txt（time x y z）')
    ap.add_argument('--bag', help='bag 目录（配合 --topic）')
    ap.add_argument('--topic', default='/lio/cloud_world')
    ap.add_argument('--split', type=float, default=SPLIT_DEFAULT, help='折返点时间戳（分腿用）')
    ap.add_argument('--max-clouds', type=int, default=0, help='只读前 N 帧（调试）')
    ap.add_argument('--vox', type=float, default=0.5, help='体素边长 m（默认 0.5）')
    ap.add_argument('--minpts', type=int, default=20, help='每体素最少点数（默认 20）')
    a = ap.parse_args()

    if a.bag:
        ts, clouds = load_bag_clouds(a.bag, a.topic, a.max_clouds)
        if not clouds:
            sys.exit(f'{a.topic} 无点云')
        npts = sum(len(c) for c in clouds)
        print('{}: {} 帧 / {} 点（{:.0f}–{:.0f}s）'.format(a.bag, len(clouds), npts,
                                                           ts[0], ts[-1]))
        out = ts < a.split
        legs = {'去程': np.concatenate([c for c, o in zip(clouds, out) if o]),
                '返程': np.concatenate([c for c, o in zip(clouds, out) if not o])}
        print('折返点 {} ⇒ 去程 {} 帧 / 返程 {} 帧'.format(
            a.split, int(out.sum()), int((~out).sum())))
        res = {}
        for lab in ('去程', '返程'):
            P = legs[lab].astype(np.float64)
            th, nz, inv, mu, cnt, uniq = voxel_stats(P, a.vox, a.minpts)
            report(lab, th, nz, a.vox, a.minpts, len(P))
            res[lab] = (uniq, mu, cnt)
        P = np.concatenate([legs['去程'], legs['返程']]).astype(np.float64)
        th, nz, inv, mu, cnt, uniq = voxel_stats(P, a.vox, a.minpts)
        report('两腿合并', th, nz, a.vox, a.minpts, len(P))
        r = cross_leg(res['去程'][0], res['去程'][1], res['去程'][2],
                      res['返程'][0], res['返程'][1], res['返程'][2], a.minpts)
        if r is None:
            print('跨腿：无两腿都够点的体素（把 --minpts 调小）')
        else:
            print('跨腿质心错位（同体素、两腿各自质心之差，n={}）: 中位 {:.3f} / P95 {:.3f} / max {:.3f} m'
                  .format(r['n'], r['dist_med'], r['dist_p95'], r['dist_max']))
            print('  分量中位: dx {:+.3f}  dy {:+.3f}  dz {:+.3f} | |dz| P95 {:.3f} m'
                  .format(r['dx_med'], r['dy_med'], r['dz_med'], r['dz_p95']))
            print('  解读: 错位 ÷ 力臂 = 腿间旋转上界；本数据集地图系 y≈沿程、x≈横向、z≈竖直')
        return

    if not a.pcd:
        print(__doc__)
        sys.exit(1)
    for f in a.pcd:
        P = load_pcd(f)
        P = P[np.isfinite(P).all(axis=1)]
        if len(P) < a.minpts:
            print(f'{f}: 点太少')
            continue
        th, nz, inv, mu, cnt, uniq = voxel_stats(P, a.vox, a.minpts)
        report(pathlib.Path(f).name, th, nz, a.vox, a.minpts, len(P))


if __name__ == '__main__':
    main()
