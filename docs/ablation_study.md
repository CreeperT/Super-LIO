# 消融实验设计

## 实验环境

- 机器人平台：Unitree B2 四足机器人
- 传感器：速腾 RS-32（前向，360° FOV）+ Livox Mid-360（后向斜装 30°）+ 内置 IMU（200 Hz）
- 测试场景：两个 rosbag
  - Bag A：长走廊（几何退化场景，Z 轴可观测性弱）
  - Bag B：楼梯（几何丰富场景，17m 高差）
- 评估指标：闭环误差（起点-终点距离）、轨迹与重定位参考的对比
- **原则：所有实验仅通过修改 `b2.yaml` 参数完成，无需外部工具或修改代码**

### 基线参数

以下参数在所有实验中**除非明确列出变动，否则保持此值**：

```yaml
# 传感器
lio.sensor.lidar_type: 2             # 速腾-32
lio.sensor.filter_rate: 3            # 每隔3点取1
lio.sensor.intensity_thresh: 80.0    # 反光柱强度阈值
lio.sensor.intensity_weight: 2.5     # 高强点额外权重
lio.sensor.voxel_fliter_size: 0.5   # 体素降采样
lio.sensor.imu_na: 0.1               # 加速度计噪声
lio.sensor.imu_ng: 0.1               # 陀螺仪噪声
lio.sensor.imu_nba: 0.001            # 加速度计零偏游走
lio.sensor.imu_nbg: 0.0005           # 陀螺仪零偏游走

# 双雷达
lio.lidar2.enable: true

# 标定
lio.calib.enable_online_calib: true

# 线特征
lio.feature.enable_edge: true
lio.feature.edge_weight: 500.0
lio.feature.high_intensity_edge_boost: 3.0

# 退化检测
lio.degeneracy.enabled: true
lio.degeneracy.max_stddev: 0.25
lio.degeneracy.htvh_cond_thresh: 30.0

# P 下界
lio.kf.p_lower_bound.enabled: false

# 动态地图
lio.relocation.update_map: false
```

---

## 实验 1：双雷达 vs 单雷达

### 目的
验证双雷达融合相对于单雷达的定位精度提升。速腾-32 提供 360° 前向视野，Mid-360 补充后向和上方视野（斜装 30°）。

### 参数修改

其余参数保持基线值不变。

| 实验组 | `lidar_type` | `lidar_topic` | `extrinsic.lidar_imu` | `lidar2.enable` | 说明 |
|:-----:|:-----:|------|------|:-----:|------|
| A1 | 2 | `/rslidar_points` | 速腾外参（基线） | `true` | 双雷达（baseline） |
| A2 | 2 | `/rslidar_points` | 速腾外参（基线） | `false` | 仅速腾-32 |
| A3 | 8 | `/livox/lidar` | Mid-360 外参 | `false` | 仅 Mid-360 |

### A3 配置

```yaml
# 将 LiDAR1 临时切换为 Mid-360
lio.sensor.lidar_type: 8              # 2 → 8 (LIVOX_PC2)
lio.ros.lidar_topic: "/livox/lidar"   # 改为 Mid-360 的话题
lio.extrinsic.lidar_imu: [-0.3, 0.0, 0.15,   # 用 Mid-360 的外参
                            0.866025, 0.0, 0.5,
                            0.0, 1.0, 0.0,
                           -0.5, 0.0, 0.866025]
lio.lidar2.enable: false              # 关闭 LiDAR2
```

### 预期

| 实验组 | Bag A 走廊 | Bag B 楼梯 |
|:-----:|------|------|
| A1 双雷达 | 稳定 | 稳定 |
| A2 仅速腾-32 | 后向/上方视野缺失，纵向约束减弱 | 仍可用（几何丰富） |
| A3 仅 Mid-360 | 非重复扫描+FOV 有限 → 特征稀疏 → **漂移** | 退化检测可能触发 |

### 指标
- 三组闭环误差对比
- Mid-360 单雷达的特征匹配率（effect_knn_num_ / ptsize 比值）

---

## 实验 2：在线标定

### 目的
验证在线外参标定在存在外参误差时的修正能力。

### 参数修改

```yaml
# 正确外参（当前使用）
lio.lidar2.extrinsic_imu: [-0.3, 0.0, 0.15,
                             0.866025, 0.0, 0.5,
                             0.0, 1.0, 0.0,
                            -0.5, 0.0, 0.866025]

# 错误外参（已注释在 b2.yaml 中，pitch 35° 替代 30°，平移 +2cm y, +5cm z）
lio.lidar2.extrinsic_imu: [-0.3, 0.02, 0.20,
                             0.819152, 0.0, 0.573576,
                             0.0, 1.0, 0.0,
                            -0.573576, 0.0, 0.819152]
```

其余参数保持基线值不变。

| 实验组 | `lidar2.extrinsic_imu` | `calib.enable_online_calib` | 预期 |
|:-----:|------|:-----:|------|
| B1 | 正确（pitch 30°） | `false` | 稳定（baseline） |
| B2 | **错误**（pitch 35°, +2cm y, +5cm z） | `false` | **返程漂移** |
| B3 | **错误**（同上） | `true` | 稳定（标定收敛后修正） |

### 指标
- 闭环误差
- `[CalibUpdate]` 日志中 `|dθ2|` 从 >1° 收敛到 ~0.001°/帧

---

## 实验 3：点-线边缘特征 + 反光柱强度加权

### 目的
验证在平面约束不足时，线特征和反光柱提供的纵向约束。

### 参数修改

| 实验组 | `lidar2.enable` | `filter_rate` | `enable_edge` | `edge_weight` | `high_intensity_edge_boost` | `intensity_thresh` | `intensity_weight` | 说明 |
|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|------|
| C1 | `true` | 3 | `true` | 500 | 3.0 | 80 | 2.5 | 基线 |
| C2 | `false` | 10 | `false` | — | — | — | — | **退化构造** |
| C3 | `false` | 10 | `true` | 500 | 1.0 | — | — | 仅线特征 |
| C4 | `false` | 10 | `true` | 500 | 3.0 | 80 | 2.5 | 线特征+反光柱 |

> `—` 表示该参数因 `enable_edge=false` 或不启用强度加权而无影响，可取任意值。

### 退化构造逻辑

通过**三个参数同时修改**来削弱平面约束，制造"需要线特征才能稳定"的条件：
1. `lio.lidar2.enable: false` — 丢失后向视野
2. `lio.sensor.filter_rate: 10` — 90% 点被丢弃，平面点密度骤降
3. 关闭线特征/反光柱权重 — 失去纵向约束

### 预期

| 实验组 | Bag A 走廊 |
|:-----:|------|
| C1 | 稳定 |
| C2 | **漂移**（平面特征不足 + 无纵向约束） |
| C3 | 部分改善（柱体/管道天然线特征提供部分纵向约束） |
| C4 | **稳定**（反光柱线特征补偿纵向约束缺失） |

---

## 实验 4：退化检测 κ+σ 四象限

### 目的
验证 κ 门槛（λmin < 500）能区分"约束真弱"和"仅不平衡"，四象限联合检测不误触发。

### 参数修改

| 象限 | 构造参数 | 预期 |
|:---:|------|------|
| κ小 σ小 | 正常配置 | 不触发，正常跟踪 |
| κ大 σ小 | 单雷达 + `filter_rate: 10` | κ INFO 告警但 λmin≥500 → 不投影 |
| κ小 σ大 | 正常配置 + `max_stddev: 0.05` | σ 检查触发，抑制不确定方向 |
| κ大 σ大 | 单雷达 + `filter_rate: 20` + Bag A | 双重触发，κ+σ 联合投影 |

### 配置表

其余参数保持基线值不变。

| 实验组 | `lidar2.enable` | `filter_rate` | `degeneracy.enabled` | `max_stddev` | `htvh_cond_thresh` | 预期 |
|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|------|
| D1 | `true` | 3 | `false` | 0.25 | 30 | 基线 |
| D2 | `false` | 10 | `true` | 0.25 | 30 | κ 大 σ 小：λmin≥500 → **不投影，稳定** |
| D3 | `false` | 20 | `true` | 0.25 | 30 | κ 大 σ 也大 → **κ+σ 联合投影** |
| D4 | `true` | 3 | `true` | **0.05** | 30 | σ 人为收紧 → σ 检查触发 |

### 指标
- `[ESKF] HTVH κ=... adequate, no projection` 日志（需开启 `verbose_logging`）
- `[ESKF] Degeneracy (σ)` 警告频率
- 闭环误差

---

## 实验 5：动态地图更新（重定位模式）

### 目的
验证运行时点云插入对先验地图的污染效应，以及 `g_update_map=false` 的正确行为。

### 参数修改

其余参数保持基线值不变。

| 实验组 | `relocation.update_map` | `enable_edge` / `high_intensity_edge_boost` | `calib.enable_online_calib` | 预期 |
|:-----:|:-----:|:-----:|:-----:|------|
| E1 | `false` | ON / 3.0 | `true` | 稳定（基线，地图静态） |
| E2 | `true` | OFF / — | `false` | **漂移**（地图被污染） |
| E3 | `true` | ON / 3.0 | `false` | 线特征补偿 → 部分改善 |
| E4 | `true` | OFF / — | `true` | 标定提高精度 → 部分改善 |
| E5 | `true` | ON / 3.0 | `true` | **稳定**（双重保护） |

### 指标
- 闭环误差
- 返程轨迹与去程轨迹的对称性

---

## 实验 6：P 下界

### 目的
验证 P 下界在滤波器过度收敛时的作用（或反作用）。

### 退化构造

通过**降低 IMU 噪声 + 增加观测点数**让 P 过度收敛：

| 实验组 | `p_lower_bound.enabled` | `filter_rate` | `imu_na` | `imu_nba` | 其余参数 | 预期 |
|:-----:|:-----:|:-----:|:-----:|:-----:|------|------|
| F1 | `false` | 3 | 0.1 | 0.001 | 基线 | 稳定 |
| F2 | `true` | 3 | 0.1 | 0.001 | 基线 | P 自然收敛 → 不碰下界 → 无影响 |
| F3 | `false` | **1** | **0.01** | **0.0001** | 基线 | P 过度收敛 → 返程"锁死"滞后 |
| F4 | `true` | **1** | **0.01** | **0.0001** | 基线 | P 钳制在 σ≥5cm → 对比 F3 |

### 指标
- `[P_lb] new min σ_trans` 日志（需 `verbose_logging`）
- 闭环误差

---

## 实验 7：SLAM vs 重定位（已知局限确认）

### 目的
定量确认 SLAM 在几何均匀环境中的 Z 轴可观测性退化，论证重定位的必要性。

### 方法

两组均使用基线参数。SLAM 启动 `b2.launch.py`（`super_lio_node`），重定位启动 `b2_relocation.launch.py`（`relocation_node`）。

| 实验组 | 启动文件 | 模式 | 参数 |
|:-----:|------|------|------|
| G1 | `b2.launch.py` | SLAM | 基线 |
| G2 | `b2_relocation.launch.py` | 重定位 | 基线 |

分别在 Bag A（走廊）和 Bag B（楼梯）上运行，对比 Z 轴轨迹。

### 预期

| 场景 | SLAM Z 范围 | 重定位 Z 范围 | SLAM 闭环 | 重定位闭环 |
|------|:---------:|:----------:|:------:|:------:|
| Bag A 走廊 | ~20m（虚假漂移） | ~3.7m（真实地形） | ~2cm | ~2cm |
| Bag B 楼梯 | ~17m（匹配真实） | ~17m | ~2cm | ~2cm |

### 论文论证
- 走廊场景：SLAM Z 轴漂移 20m（不可观测），重定位全程精确（先验地图提供绝对参考）
- 楼梯场景：两者相当（几何丰富使 Z 轴可观测）
- 两种模式闭环精度均 <2cm，但 SLAM 中途轨迹严重偏离
- 曾尝试四种地面软约束方案均失败，证明"后验约束无法弥补可观测性缺失"

---

## 指标量化方法

### 通用流程

```bash
# 1. 运行实验 + 录制轨迹
ros2 bag play <bag> --clock
ros2 bag record -o exp_N_group /lio/odom

# 2. 导出 TUM 格式轨迹
python3 tools/export_traj.py exp_N_group/ exp_N_group.txt

# 3. 闭环误差（返程场景必需）
python3 tools/export_traj.py --loop exp_N_group.txt

# 4. 与重定位参考对比（如果有）
evo_ape tum reloc_ref.txt exp_N_group.txt --plot --plot_mode xyz
```

### 各实验专属指标

#### 实验 1：双雷达 vs 单雷达

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **闭环误差** | `export_traj.py --loop` | 越小越好，三组对比 |
| **Z 轴范围** | `awk '{print $4}' traj.txt \| awk 'BEGIN{min=999;max=-999}...'` | 虚假漂移量，A1/A2 应 <5m，A3 可能更大 |
| **特征匹配率** | 日志中 `effect_knn_num_` / `ptsize` | A3 应显著低于 A1/A2 |

#### 实验 2：在线标定

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **闭环误差** | `export_traj.py --loop` | B2 应远大于 B1/B3 |
| **外参收敛速度** | 日志 `[CalibUpdate]` 中 `\|dθ2\|` 从 >1° → <0.01° 的帧数 | B3 应在前 50 帧内收敛 |
| **外参终值偏差** | 日志 `[Calib#...] ext2: rpy=[...]°` 末帧值 vs 真值 | B3 应收敛到接近真值（pitch≈30°） |

#### 实验 3：线特征 + 反光柱权重

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **闭环误差** | `export_traj.py --loop` | C2 漂移 → 闭环误差大；C4 恢复 → 闭环误差小 |
| **线特征点数** | 日志或 `edge_mask_` 统计 | C3/C4 线特征点占比 |
| **纵向漂移** | 轨迹沿走廊方向的分量与重定位参考的差值 | C4 反光柱提供纵向约束 → 纵向漂移小 |

#### 实验 4：退化检测

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **κ 告警频率** | `grep "HTVH κ=" log.txt \| wc -l` | D2 应零告警（λmin 门槛挡住） |
| **σ 退化触发次数** | `grep "Degeneracy (σ)" log.txt \| wc -l` | D3 应 >0；D1/D2 应为 0 |
| **退化期间轨迹漂移** | D2 vs D3 闭环误差对比 | D2 不投影但告警 → 闭环误差应仍小 |

#### 实验 5：动态地图更新

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **闭环误差** | `export_traj.py --loop` | E2 应最大，E5 应接近 E1 |
| **HTVr 范数变化** | `grep "HTVr" log.txt`（需加日志） | E2 去程 HTVr 逐渐增大（地图被污染） |

#### 实验 6：P 下界

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **min σ_trans** | `grep "\[P_lb\]" log.txt`（需 `verbose_logging: true`） | F3 应降至 <1mm；F4 应被钳在 5cm |
| **闭环误差** | `export_traj.py --loop` | F3 vs F4 对比 |
| **返程初始段滞后** | 返程前 5 秒轨迹与重定位参考的差值 | F3 "锁死" → 返程初段滞后 > F4 |

#### 实验 7：SLAM vs 重定位

| 指标 | 获取方式 | 含义 |
|------|------|------|
| **Z 轴范围** | SLAM vs 重定位轨迹 Z 的 min/max | 走廊：SLAM ~20m vs 重定位 ~3.7m |
| **闭环误差** | `export_traj.py --loop` | 两者均应 <5cm |
| **全程 ATE** | `evo_ape tum reloc.txt slam.txt` | 重定位作为参考，SLAM 的 ATE 应在走廊大幅高于楼梯 |

### 结果汇总表模板

| 实验 | 组 | 闭环误差 (m) | Z 范围 (m) | 专属指标 | 结论 |
|------|:--:|:---------:|:--------:|------|------|
| 1 | A1 | | | 特征匹配率: | |
| 1 | A2 | | | | |
| 1 | A3 | | | | |
| 2 | B1 | | | 外参收敛帧数: | |
| 2 | B2 | | | | |
| 2 | B3 | | | | |
| ... | ... | ... | ... | ... | ... |

---

## 消融矩阵总览

```
          双雷达  标定  线特征  κ门槛  地图静态  P下界
实验1      ██
实验2             ██
实验3      ██     ██   ██
实验4      ██                 ██
实验5                          ██      ██
实验6                                         ██
实验7      ██           (SLAM vs 重定位对比)
```
