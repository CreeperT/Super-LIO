# CHANGELOG

## 2026-07-06: 返程漂移根因定位 + 修复

### 返程漂移根因
经过逐项排除，定位到两个独立原因：

**1. Mid-360 外参标定误差（主因）**
- 手动测量的 Mid-360 外参存在小角度误差
- 去程建图时 Mid-360 点在错误位置写入地图
- 返程时速腾扫描匹配到地图中偏移的 Mid-360 点 → ICP 产生系统性偏差 → 逐帧累积 → 跑飞
- **修复**：开启在线标定 `lio.calib.enable_online_calib: true`，ESKF 从 ICP 残差中自动修正外参
- 速腾-32 前向被遮挡的场景下，Mid-360 的点在返程时成为关键约束，外参误差的影响被放大

**2. P 下界防过收敛导致系统性漂移（次因）**
- 7/5 添加的 P 下界（σ≥5cm/30cm）强制 P 不低于阈值
- ICP 在弱约束方向的噪声修正每次都能渗透 → 逐帧累积偏移
- 同事版本（6/24 + 强度加权）无 P 下界，P 可自然收敛到极小值 → 噪声被抑制 → 无漂移
- **修复**：注释掉 `Predict()` 和 `UpdateObserve()` 中的 P 下界

### 高强点线特征增强
- 反光柱（强度>80）优先提取线特征（跳过平面），提供走廊纵向约束
- 高强线特征权重额外 ×3（总权重 = 500 × 强度权重 × 3）
- 新增参数 `lio.feature.high_intensity_edge_boost`（默认 3.0）
- 涉及：`params.h/cpp`、`ROSWrapper.cpp`、`super_lio.cpp`、`b2.yaml`

### 强度过滤覆盖全部 LiDAR Handler
- 所有有效 handler（LIVOX×2、HESAI16 PC2×2、VELO×2、OUSTER×2、LS16/LIVOX_PC2×2）均添加高强点不过滤逻辑
- VEL_NCLT（固定 intensity=1.0）跳过

### 参数调整
- `lio.calib.enable_online_calib`: false → true
- `lio.degeneracy.max_stddev`: 0.5 → 0.25
- `lio.degeneracy.htvh_cond_thresh`: 50 → 30
- `lio.kf.kf_max_iterations`: 5 → 6
- `lio.kf.kf_quit_eps`: 0.001 → 0.0001

**涉及文件**: `ESKF.cpp`（P 下界注释）、`super_lio.cpp`（高强线特征）、`params.h/cpp`、`ROSWrapper.cpp`、`b2.yaml`

---

## 2026-07-05: 定位鲁棒性增强 — 退化恢复 + P下界 + 强度加权 + 地图冲突检测

### 退化恢复状态机
- `ESKF.h/cpp` `RemapDegeneracy()`: 新增退化恢复检测—κ 下降后连续5帧确认→进入恢复模式(10帧)，跳过dx投影，P平移块膨大到1.0m²
- 退化期间冻结bg/ba更新—无观测方向只含噪声
- 涉及: `ESKF.h` (was_degenerate_, recovery_confirm_count_, recovery_remaining_), `ESKF.cpp`

### P下界防过收敛
- `ESKF.cpp` `Predict()` / `UpdateObserve()`: 平移P下界 σ≥5cm (P=2.5e-3)，旋转P下界 σ≥0.1° (P=3e-6)
- 退化时平移P下界自动提升至 σ≥30cm (P=0.09)，增大观测修正空间
- 防止ESKF过度信任IMU→位置/旋转"锁死"→拐角处点云偏移

### 地图冲突检测 + 局部重建
- `ESKF.cpp` `UpdateObserve()`: |HTVr|突增>5000持续20帧→触发地图重建
- `RemapDegeneracy()`: κ退化持续100帧→触发地图重建
- `super_lio.cpp` `stateProcess()`: 清图后skip_observe_frames_=5→先累积5帧地图点再恢复IEKF
- 涉及: `ESKF.h` (reset_map_requested_, consecutive_degen_frames_)

### 反光柱强度加权
- 参数: `lio.sensor.intensity_thresh` (默认200→80), `lio.sensor.intensity_weight` (默认2.0)
- `ROSWrapper.cpp`: LiDAR handler遍历全部点，高强点不受filter_rate限制
- `super_lio.cpp` `Observe()`: 高强点权重线性提升至 1+g_intensity_weight 倍
- 涉及: `params.h/cpp`, `ROSWrapper.cpp` (HESAI16/LIVOX_PC2×2), `super_lio.cpp`, `ESKF.h` (calib_s0_)

### 动态数组化
- `super_lio.h`: effect_mask_/edge_mask_/effect_knn_mask_ 从固定bool[40000]→vector\<char\>
- `super_lio.cpp` `Observe()`: abcd_vec_/line_dir_vec_等每帧按ptsize动态resize

**涉及文件**: `ESKF.h`, `ESKF.cpp`, `super_lio.h`, `super_lio.cpp`, `params.h`, `params.cpp`, `ROSWrapper.cpp`, `b2.yaml`

---

## 2026-06-25: Nav2 TF — map→odom→base_link 发布

**功能**：新增 Nav2 导航所需的 TF 树：
- `map → odom`: 恒等变换（LIO 全局定位，无漂移）
- `odom → base_link`: LIO 机器人位姿（含 IMU→base_link 外参）

**配置**：`lio.output.nav2_tf: true`

**涉及文件**：`params.h`, `params.cpp`, `ROSWrapper.cpp`, `b2.yaml`

---

## 2026-06-18: 在线标定修复 — 符号、Lidar2检测、块对角、长度归一化

**问题**：
1. 外参旋转雅可比符号错误：`∂r/∂δθ = -(n×p)ᵀ = (p×n)ᵀ`，写成了 `n×p`
2. `from_lidar2 = (_weights[idx] != 1.0f)` — `g_lidar2_weight=1.0` 时判断失效，lidar2 点永远不会被标定
3. 标定块与位姿块耦合导致 33×33 矩阵条件数爆炸
4. 去畸变用固定 `g_lidar_imu`，雅可比用 KFState 估计值 → 不一致

**修复**：

### 符号修复
- `super_lio.cpp`: `J_ext_rot = p_raw.cross(nb_lidar)` 替代 `nb_lidar.cross(p_raw)`

### Lidar2 检测修复
- 新增 `_is_lidar2` 数组，用 intensity 符号直接判断，不依赖 weight 值

### 块对角近似
- `ESKF.cpp`: 标定更新从 33×33 联合求逆中分离，在 IEKF 收敛后独立求解 15×15 系统
- 位姿更新不受标定块影响，数值稳定

### 雅可比长度归一化
- `super_lio.cpp`: 计算 `s0 = median(|p_body|)`，`J_ext_rot /= s0` 消除量纲差异
- 求解后 `dθ /= s0` 恢复真实修正量

### 去畸变自适应
- `super_lio.cpp` `Propagation_Undistort`: 使用 `kf_->getExtrinsic1/2()` 替代固定 `g_lidar_imu`
- 雅可比使用 `kf_state.T_L1_IMU/T_L2_IMU` → 全链路自洽

**涉及文件**：`super_lio.cpp`, `super_lio_reloc.cpp`, `ESKF.cpp`, `ESKF.h`

---

## 2026-06-17: Phase 1 — 在线时空标定（ESKF 状态扩展 + 外参雅可比）

**论文方向**：在线时空标定 (Online Spatiotemporal Calibration)

**改动**：

### ESKF 状态扩展 (18→33 维)
- `ESKF.h`: 状态从 `[dr,dp,dv,dbg,dba,dg]`(18D) 扩展为包含:
  - 外参1: `[dθ1, dρ1]` (6D SE(3) 右扰动)
  - 外参2: `[dθ2, dρ2]` (6D)
  - 时偏: `[dt1, dt2, pad]` (3D, 当前未估计)
  - 总计 33D
- 新增 `KFState` 包含外参和时偏名义值
- 新增 `OBS_MAT` (21x21), `OBS_VEC` (21x1) 用于观测雅可比

### 外参雅可比推导
- `super_lio.cpp` `Observe()` lambda:
  - 每点根据 intensity 符号判断 LiDAR 来源
  - 计算 `p_raw = T_ext^{-1} * p_body`（从 IMU 系反推到 LiDAR 系）
  - 计算 J_ext_rot = nb_lidar × p_raw（旋转雅可比）
  - 计算 J_ext_trans = nb_lidar（平移雅可比）
  - 构建 21D 全雅可比向量 J = [J_pose(6), J_ext(6), 0(6), J_dt(3)]
  - 累加到 21x21 HTVH_calib 和 21x1 HTVr_calib

### 过程噪声 + P 初始化
- 外参旋转初始不确定性: σ=3°, P=3.0e-3·I
- 外参平移初始不确定性: σ=10cm, P=0.01·I
- 时偏初始不确定性: σ=5ms, P=2.5e-5
- 随机游走噪声: 旋转 1.7e-4 rad/√s, 平移 1e-4 m/√s, 时偏 1e-6 s/√s
- 可通过 yaml 调整

### 开关控制
- `lio.calib.enable_online_calib: true/false` — 一键开关在线标定
- `ESKF::setCalibEnabled()` — 运行时切换

### 涉及文件
- `include/lio/ESKF.h` — 状态扩展、接口
- `src/lio/ESKF.cpp` — SetInitialConditions, Update, Predict, UpdateObserve
- `src/lio/super_lio.cpp` — 外参雅可比计算、标定状态日志
- `src/lio/super_lio_reloc.cpp` — setCalibEnabled
- `include/lio/params.h`, `src/lio/params.cpp` — 新参数
- `src/ros/ROSWrapper.cpp` — 参数加载
- `config/b2.yaml` — 新配置段

### before/after 对比
- 设置 `lio.calib.enable_online_calib: false` → 等同于改前行为
- 设置 `lio.calib.enable_online_calib: true` → 启用在线标定
- 终端每 100 帧打印当前外参估计值

---

## 2026-06-17: 修复小体素降采样退化 + 退化检测漏检

**问题**：`voxel_fliter_size` < 0.3m 时发生明显定位退化，且退化检测未触发。

**根因分析**（详见对话）：
1. **数学退化（主因）**：冗余点不对称放大 HTVH → 条件数 κ 极端恶化；P⁻¹ 先验掩盖未约束方向。
2. **数组越界（叠加）**：固定 20000 元素数组，小体素时 ptsize 超限 → 内存损坏。
3. **退化检测只查 σ 不查 κ**：后验方差 σ 被先验压制而 HTVH 条件数已爆炸。

**修复 A — 数组完全动态化**：
- `effect_mask_`, `effect_knn_mask_`, `edge_mask_`: 从固定 `bool[40000]` → `std::vector<char>`，每帧按 ptsize resize
- `abcd_vec_`, `line_dir_vec_`, `line_center_vec_`, `effect_knn_idxs_`: 移除 init() 中固定 20000 resize，改在 Observe() 开头动态 resize
- 涉及：`super_lio.h`, `super_lio.cpp`, `super_lio_reloc.cpp`

**修复 B — 退化检测增加 HTVH 条件数检查**：
- 新增 `DegeneracyOptions::htvh_cond_thresh` (默认 100)
- `RemapDegeneracy` 同步检查两个条件：
  1. 后验 σ > `max_stddev`（原有）
  2. HTVH[3:6,3:6] 条件数 κ > `htvh_cond_thresh`（**新增**）
- κ 超标时：抑制 HTVH 最小特征值方向（约束最弱方向）的 dx
- 涉及：`ESKF.h`, `ESKF.cpp`
- 新参数：`g_degeneracy_htvh_cond_thresh`, `lio.degeneracy.htvh_cond_thresh`

**修复 C — voxel_fliter_size 下限警告**：
- `ROSWrapper.cpp` 加载参数后：若 < 0.3m 输出 WARNING

**涉及文件**：`super_lio.h`, `super_lio.cpp`, `super_lio_reloc.cpp`, `ESKF.h`, `ESKF.cpp`, `params.h`, `params.cpp`, `ROSWrapper.cpp`, `b2.yaml`, `relocation.yaml`

---

## 2026-06-16: 修复 spinSensorOnce 在 rclcpp::spin 内调用的崩溃

**问题**：`kf_init()` 成功后调用 `spinSensorOnce()` 时崩溃：
```
Callback group has already been added to an executor.
```

**根因**：`spinSensorOnce()` 创建新的 `SingleThreadedExecutor` 尝试添加 `cb_sensor_`，但 `cb_sensor_` 已经关联到 `rclcpp::spin()` 创建的 executor。ROS 2 不允许一个 callback group 同时关联多个 executor。

**修复**：`spinSensorOnce()` 在添加 `cb_sensor_` 到本地 executor 前，先通过 `get_associated_with_executor_atomic().exchange(false)` 临时释放关联，spin 完成后恢复。由于调用发生在主 executor 的回调内（主 executor 阻塞等待），不存在竞争条件。

**涉及文件**：`src/super_lio/include/ros/ROSWrapper.h` — `spinSensorOnce()`

---

## 2026-06-16: 回退边缘特征分类到简单逻辑

**变更**：移除 PCA 预分类（linearity/planarity/sphericity 判断 prefer_edge），回归最简单的逻辑：
1. 对所有点先尝试平面拟合
2. 平面拟合失败的点中，再尝试线拟合作为边缘特征

**删除的参数**：`g_edge_linearity_thresh`、`g_edge_sphericity_thresh`、`g_edge_max_planarity`、`g_edge_scan_dist_thresh`

**保留的参数**：`g_enable_edge_feature`、`g_edge_weight`、`g_edge_line_dist_thresh`、`g_edge_linearity_ratio`

**涉及文件**：
- `src/super_lio/src/lio/super_lio.cpp` — PCA 分类块替换为简单平面优先逻辑
- `src/super_lio/include/lio/params.h` — 移除 4 个废弃参数声明
- `src/super_lio/src/lio/params.cpp` — 移除 4 个废弃默认值
- `src/super_lio/src/ros/ROSWrapper.cpp` — 移除 4 个参数加载
- `config/b2.yaml`、`config/relocation.yaml` — 清理配置

---

## 2026-06-16: 修复重定位初始位姿解算不稳定的三个问题

**问题**：初始位姿解算有时成功有时失败。

**根因与修复**：

### 修复 1：重力估计受运动加速度污染
- **根因**：`kf_init()` 使用所有 IMU 加速度的滑动平均来估计重力方向。如果累积 3 帧期间机器人有运动（哪怕是微小晃动），平均加速度就不等于纯重力 → roll/pitch 估计错误 → ICP 发散。
- **修复**：仅使用静止状态的 IMU 数据进行重力估计。判断标准：`|acc|` 与 `g_gravity_norm` 偏差 < 1.5 m/s² 且 `|gyr|` < 0.15 rad/s。

### 修复 3：双雷达点云用错外参
- **根因**：lidar1 和 lidar2 的原始点云混在同一容器中，统一用 `g_lidar_imu` 变换到 IMU 系。lidar2（Mid-360）的点应该用 `g_lidar_imu2` 变换。这导致 lidar2 的点出现在错误位置，给 ICP 输入"重影"点云。
- **修复**：分别累积 lidar1/lidar2 点云（`init_obs_data_` / `init_obs_data2_`），各自用对应外参变换后在 IMU 系合并。

### 修复 4：静止检测警告
- **新增**：累积完成后检查静止 IMU 比例和平均角速度。若 < 50% 静止或平均角速度 > 0.1 rad/s，输出 WARNING 日志提醒用户保持机器人静止。

**涉及文件**：
- `src/super_lio/include/lio/super_lio_reloc.h` — 新增 `init_obs_data2_` 成员
- `src/super_lio/src/lio/super_lio_reloc.cpp` — 静止检测、分离累积、分离变换

---

## 2026-06-16: 收紧点-线边缘特征分类逻辑

**问题**：边缘特征点（绿色）大量混杂在平面特征点（蓝色）中，尤其在拱顶等曲面区域产生假边缘。

**根因**：
1. PCA 分类只用 `linearity = (λ2-λ1)/λ2` 一个指标，未检查 λ0 — 曲面（拱顶）的 λ2 可能碾压 λ1 使 linearity > 0.7，但 λ0 实际不小
2. `linearity > planarity × 2` 太宽松，噪声平面可能满足
3. `g_edge_line_dist_thresh = 0.2`（20cm）太松，KNN 邻居到拟合线距离阈值过大
4. 扫描点到拟合线使用与邻居点相同阈值，应该更严格

**修改**：

| 改动 | 说明 |
|------|------|
| 新增 `g_edge_sphericity_thresh` (0.05) | λ0/λ2 < 此值，确保局部几何是"扁"的而非曲面 |
| 新增 `g_edge_max_planarity` (0.3) | planarity < 此值，排除明显平面 |
| 新增 `g_edge_scan_dist_thresh` (0.05) | 扫描点到拟合线独立距离阈值(5cm) |
| `linearity > planarity × 2` → `× 3` | 更严格区分线与面 |
| `g_edge_linearity_thresh` 0.7 → 0.8 | 要求更主导的线方向 |
| `g_edge_line_dist_thresh` 0.2 → 0.08 | 邻居点必须紧贴拟合线(8cm) |

**prefer_edge 新条件**（全部满足才进入边缘优先路径）：
1. `linearity > 0.8`（λ2 碾压 λ1）
2. `linearity > planarity × 3`（不是平面伪装）
3. `λ0/λ2 < 0.05`（不是曲面/拱顶）
4. `planarity < 0.3`（局部不能太像平面）

**涉及文件**：
- `src/super_lio/include/lio/params.h` — 新增 3 个参数声明
- `src/super_lio/src/lio/params.cpp` — 新增 3 个默认值
- `src/super_lio/src/lio/super_lio.cpp` — PCA 分类逻辑：加入 sphericity/planarity 上限、比值收紧、扫描点距离用独立阈值
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载 3 个新参数
- `config/b2.yaml` — 更新参数值及中文注释
- `config/relocation.yaml` — 同步更新

---

## 2026-06-15: 修复双雷达 sync_measure has_lidar2 被反复重置

**问题**：`/lio/cloud_world` 只有雷达1的点云，雷达2点云缺失。

**根因**：`sync_measure()` 中 `meas.has_lidar2 = false` 在每次调用时执行。lidar2 匹配成功后 buffer 被消费清空，后续调用将 `has_lidar2` 重置为 `false`。

**修复**：将 lidar2 匹配逻辑移入 `if (!lidar_pushed_)` 块内，与雷达1帧获取同步——只在获取新雷达1帧时才重置和匹配雷达2。

**涉及文件**：`src/super_lio/src/ros/ROSWrapper.cpp` — `sync_measure()`

---

## 2026-06-15: 新增 LIVOX_PC2 LiDAR 类型 + lidar2 时钟偏移修正

**功能**：
1. 新增 `LIVOX_PC2` (type=8) — 支持 Livox Mid-360 以标准 PointCloud2 格式 (x,y,z,intensity,tag,line,timestamp) 输出
2. 新增 `lio.lidar2.time_offset` — 修正 lidar2 与 IMU/雷达1 的时钟偏差

**LIVOX_PC2 PCL 点类型** (`basic/alias.h`):
```cpp
namespace livox_pc2 {
struct Point { float x,y,z, intensity; uint8_t tag, line; double timestamp; };
}
```
- tag 过滤: 保留 `0x10`(双回波) 和 `0x00`(单回波)
- timestamp: 绝对时间戳 → offset_time = pt.timestamp - time_begin

**涉及文件**：
- `src/basic/include/basic/alias.h` — 新增 `livox_pc2::Point` + PCL 注册
- `src/super_lio/include/common/ds.h` — `LIVOX_PC2=8`, `LID_TYPE_NAMES` 扩展
- `src/super_lio/src/ros/ROSWrapper.cpp` — `stdMsgHandler`/`stdMsgHandler2` 各新增 LIVOX_PC2 分支
- `src/super_lio/include/lio/params.h` / `params.cpp` — `g_lidar2_time_offset`
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — `lidar_type:8`, `time_offset`

---

## 2026-06-15: 双雷达数据级融合 (Dual-LiDAR Fusion)

**功能**：在 ESKF 观测层面实现 RoboSense-32（前向）和 Mid-360（后向斜装30°）的数据级耦合，替代话题层粗暴糅合。

**架构**：
- 双独立回调 (livoxHandler2 / stdMsgHandler2) 分别接收两个雷达点云
- sync_measure() 以雷达1时间窗口为基准，匹配雷达2的重叠帧
- Propagation_Undistort() 用各自外参独立去畸变后合并在 IMU 坐标系
- 合并后统一下采样、统一 Observe → 同一 HTVH/HTVr → ESKF 一次性更新

**新增参数** (`lio.lidar2.*`):
- `enable`, `lidar_type`, `lidar_topic`, `blind`, `maxrange`, `filter_rate`, `weight`
- `extrinsic_imu`: 12 值 (tx, ty, tz, R00..R22)

**涉及文件**：
- `include/lio/params.h`, `src/lio/params.cpp` — 新增 lidar2 参数
- `include/common/ds.h` — MeasureGroup 扩展 (lidar2, has_lidar2)
- `include/ros/ROSWrapper.h` — 双订阅、双缓冲区、lidar2 回调声明
- `src/ros/ROSWrapper.cpp` — 双回调、sync_measure 改造、参数加载
- `include/lio/super_lio.h` — scan_undistort_full_2_、数组扩容至 40000
- `src/lio/super_lio.cpp` — Propagation_Undistort 双雷达去畸变+合并
- `src/lio/super_lio_reloc.cpp` — 重定位双雷达点云合并
- `config/b2.yaml`, `config/relocation.yaml` — lidar2 配置

---

## 2026-06-12: IMU 噪声参数调整 + relocation.yaml typo 修复

**问题**：z 轴漂移在 ba 初始化修复后仍然存在。

**根因**：
1. `imu_nba=0.0001` 过小：每步传播 P_ba 仅增长 `1e-8`，卡尔曼增益中 ba 修正量近乎为零，滤波后 P_ba 迅速收缩至零，后续 ba 残差无法被修正
2. relocation.yaml 中 `lio.sensor.nbg` typo（缺 `imu_` 前缀），导致 `imu_nbg` 参数从未生效

**参数调整**：
| 参数 | 旧值 | 新值 | 说明 |
|------|------|------|------|
| `imu_na` | 0.1 | 0.05 | 降低加速计噪声，增强 LiDAR 观测相对权重 |
| `imu_ng` | 0.1 | 0.01 | 降至合理值（现代 IMU: 0.001-0.01） |
| `imu_nba` | 0.0001 | 0.01 | **100x** — 允许 ba 在滤波后维持足够不确定性以持续修正 |
| `imu_nbg` | 0.0001 | 0.0005 | 5x — 与 nba 保持合理比例 |

**涉及文件**：
- `src/super_lio/config/b2.yaml` — 更新 4 个 IMU 参数
- `src/super_lio/config/relocation.yaml` — 同上 + 修复 typo `nbg` → `imu_nbg`

---

## 2026-06-12: 修复加速度计零偏未初始化导致的 z 轴漂移

**问题**：重定位初始矩阵正确但有概率发生 z 轴负方向偏移，偏移大则完全发散，偏移小则整体轨迹向下偏移。

**根因**：`kf_init()` 中加速度计零偏 `ba_` 始终初始化为 0，且 `SetX()` 覆盖了 `SetInitialConditions()` 设置的 `bg_`。零偏残差在 ESKF 传播中累积为 z 方向位置误差。

**修复**：
- 从静止初始化数据估计零偏：`estimated_ba = mean_acce * (1 - imu_scale)`
- 在 `SetX` 前设置 `state.bg` 和 `state.ba`，防止被覆盖
- 增大 `ba_` 初始协方差至 `1e-2*I` (σ=0.1 m/s²)，加快残差收敛

**涉及文件**：
- `src/super_lio/src/lio/super_lio.cpp` — `kf_init()`：估计并传递 ba/bg
- `src/super_lio/src/lio/super_lio_reloc.cpp` — `kf_init()`：同上
- `src/super_lio/src/lio/ESKF.cpp` — `SetInitialConditions()`：增大 ba 初始协方差

---

## 2026-06-11: 退化检测改用后验协方差标准差

**问题**：退化检测在楼梯上误判（相对特征值比值 < 0.05），解映射砍掉有效约束导致漂移。

**修复**：改用后验协方差 `Q_trans` 的物理标准差（米）判断退化，`σ > max_stddev(0.5m)` 才触发解映射。

**涉及文件**：
- `src/super_lio/include/lio/ESKF.h` — `DegeneracyOptions.max_trans_stddev` 替代 `eigen_threshold`
- `src/super_lio/src/lio/ESKF.cpp` — `RemapDegeneracy()`：分析后验协方差替代 HTVH
- `src/super_lio/include/lio/params.h` / `params.cpp` — `g_degeneracy_max_stddev` (0.5m)
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载 `lio.degeneracy.max_stddev`
- `src/super_lio/src/lio/super_lio.cpp` / `super_lio_reloc.cpp` — 更新选项名
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — 更新参数

---

## 2026-06-11: 增加点-线（边缘）特征

**功能**：在 OctVoxMap 平面拟合失败后尝试用 PCA 拟合直线，对成功拟合为线的点添加点-线残差约束。

**参数**：`lio.feature.enable_edge`, `edge_weight`(100), `edge_line_dist_thresh`(0.05m), `edge_linearity_ratio`(5.0)

**涉及文件**：
- `src/super_lio/include/lio/params.h` / `params.cpp` — 新增边缘特征参数
- `src/super_lio/include/lio/super_lio.h` — 新增 `edge_mask_`, `line_dir_vec_`, `line_center_vec_`
- `src/super_lio/src/lio/super_lio.cpp` — `fit_line_from_points()`, 修改 `Observe()`
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载参数
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — 添加配置项

---

## 2026-06-11: 增加退化检测与解映射

**功能**：对观测信息矩阵做特征分解，退化方向限制状态更新。

**涉及文件**：
- `src/super_lio/include/lio/ESKF.h` — `DegeneracyOptions` 结构体, `RemapDegeneracy()`
- `src/super_lio/src/lio/ESKF.cpp` — `RemapDegeneracy()` 实现，在 `UpdateObserve()` 调用
- `src/super_lio/include/lio/params.h` / `params.cpp` — 退化参数
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载参数
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — 添加配置项

---

## 2026-06-11: 重定位鲁棒性增强

**功能**：
- 累积帧数可配置 (`lio.relocation.init_frames`, 默认 5)
- z-offset 搜索（±2m, 步长 1m）
- 地图 z-bounds 检查
- ICP 重叠率验证 (`lio.relocation.min_overlap_ratio`, 默认 0.3)

**涉及文件**：
- `src/super_lio/include/lio/params.h` / `params.cpp` — 新增重定位鲁棒性参数
- `src/super_lio/include/lio/super_lio_reloc.h` — 新增 `map_z_min_/max_`
- `src/super_lio/src/lio/super_lio_reloc.cpp` — z-bounds 计算、z-offset 搜索、重叠率检查
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载参数
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — 添加配置项

---

## 2026-06-11: 修复 SuperLIOReLoc::init() 缺少初始化导致 SIGSEGV

**问题**：`SuperLIOReLoc::init()` 完全覆盖基类，缺少 `line_dir_vec_`/`line_center_vec_` resize 和 degeneracy options 设置，导致崩溃。

**修复**：在子类 `init()` 中补充缺失的初始化。

**涉及文件**：
- `src/super_lio/src/lio/super_lio_reloc.cpp` — 补充向量 resize 和 degeneracy 设置

---

## 2026-06-11: 实现 6-DOF interactive marker (C++)

**功能**：使用 `interactive_markers::InteractiveMarkerServer` 创建 6-DOF 初始位姿手柄，发布 `/initialpose`。

**涉及文件**：
- `src/super_lio/src/apps/initial_pose_3d_marker.cpp` — 节点实现
- `src/super_lio/launch/b2_relocation.launch.py` — 启动节点
- `src/super_lio/rviz/relocation.rviz` — rviz 配置
- `src/super_lio/CMakeLists.txt` — 编译配置

---

## 2026-06-11: 重定位观测可视化 + 3D 位姿支持

**功能**：
- `pub_init_obs()` 发布初始观测点云到 `/lio/init_obs`
- `/initialpose` 订阅支持 3D pose（z 坐标来自消息而非硬编码）

**涉及文件**：
- `src/super_lio/include/ros/ROSWrapper.h` / `src/super_lio/src/ros/ROSWrapper.cpp`
- `src/super_lio/src/lio/super_lio_reloc.cpp`

---

## 2026-06-11: 多分辨率 NDT 级联重定位

**功能**：粗(5m)→中(2m)→精(ICP) 三级配准，支持多 yaw 搜索回退。

**涉及文件**：
- `src/super_lio/src/lio/super_lio_reloc.cpp` — 多分辨率级联实现
- `src/super_lio/include/lio/params.h` / `params.cpp` — 配准参数
- `src/super_lio/src/ros/ROSWrapper.cpp` — 加载参数
- `src/super_lio/config/b2.yaml` / `relocation.yaml` — 配置
