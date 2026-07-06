# Adaptive Multi-LiDAR Inertial Odometry: 论文工作实施路线图

## 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│  Phase 1: 在线时空标定 (Online Spatiotemporal Calibration)   │
│  ESKF 状态扩展: [R,p,v,bg,ba] → [R,p,v,bg,ba,T_ext1_imu,  │
│                                    T_ext2_imu, δt1, δt2]   │
│  24维 → 38维                                                │
├─────────────────────────────────────────────────────────────┤
│  Phase 2: Per-LiDAR Fisher 信息分析                         │
│  分离两个雷达的 HTVH/HTVr → 特征谱分析 → 互补度量           │
├─────────────────────────────────────────────────────────────┤
│  Phase 3: 退化感知的自适应融合                               │
│  Fisher 条件数 → 退化检测 → IMU主导切换 → 自适应权重         │
├─────────────────────────────────────────────────────────────┤
│  Phase 4: 结构化楼梯边缘约束                                  │
│  平行/等间距边缘检测 → pseudo-measurement 残差               │
├─────────────────────────────────────────────────────────────┤
│  Phase 5: 实验评估 + 论文写作                                 │
│  消融实验 / 公开数据集 / B2数据集 / 写作                     │
└─────────────────────────────────────────────────────────────┘
```

### 论文叙事线

```
在线标定 → 准确外参
    ↓
Per-LiDAR Fisher 信息 → 互补度量 → 自适应权重
    ↓
Fisher 条件数 → 退化检测 → IMU主导模式切换
    ↓
退化恢复 + 结构边缘 → 多楼层鲁棒定位
```

---

## Phase 1: 在线时空标定 (2-3 周)

### 目标
将 LiDAR-IMU 外参 SE(3) 和时间偏移 δt 作为 ESKF 状态在线估计，
消除离线标定的依赖。

### 1.1 ESKF 状态扩展

**文件**: `src/super_lio/include/lio/ESKF.h`

将状态维度从 18 扩展到 38（或更灵活的定义）：

```cpp
// 当前: [R(3), p(3), v(3), bg(3), ba(3)] = 18 维误差状态
// 新增: [T_L1_IMU(6), T_L2_IMU(6), δt1(1), δt2(1)] = 14 维

static const int STATE_DIM = 18;
static const int CALIB_DIM = 14;   // 外参+时偏
static const int FULL_DIM = 32;    // STATE_DIM + CALIB_DIM

struct STATE {
  // ... existing ...
  BASIC::SE3 T_L1_IMU;  // LiDAR1→IMU 外参
  BASIC::SE3 T_L2_IMU;  // LiDAR2→IMU 外参
  double delta_t1;       // LiDAR1 时间偏移
  double delta_t2;       // LiDAR2 时间偏移
};
```

**关键设计决策**:
- 外参使用 SE(3) 右扰动模型（与现有 SO(3) 扰动一致）
- 时间偏移使用标量随机游走模型
- 外参初始值从 yaml 加载，初始方差较大（σ=1°, σ=5cm）
- 时间偏移初始方差 = 0.01s²（对应 ~10ms 不确定性）

### 1.2 外参雅可比推导

**文件**: `src/super_lio/src/lio/ESKF.cpp` — 在 `UpdateObserve()` 中扩展

**点到面残差关于外参的雅可比**:

```
残差: r = nᵀ(p_w + d)
其中 p_w = R * (T_ext * p_raw) + t
         = R * (R_ext * p_raw + t_ext) + t

∂r/∂δθ_ext:  推导使用 SE(3) 右扰动
∂r/∂δt_ext:  nᵀ * R
```

**关键公式（点-面残差）**:

```
J_θ_ext = nᵀ * R * [p_body]×    // p_body = R_ext * p_raw + t_ext
J_t_ext = nᵀ * R                 // 3×1 平移雅可比 = 平面法向在 world 系投影
```

**关键公式（点-线残差）**:

```
J_θ_ext = n_rᵀ * R * [p_body]×
J_t_ext = n_rᵀ * R
```

**时间偏移雅可比**:

```
∂p_raw/∂δt = v_scan  // 扫描点的瞬时速度（由IMU插值得到）
∂r/∂δt = nᵀ * R * R_ext * v_scan
```

### 1.3 外参过程噪声设计

**文件**: `src/super_lio/src/lio/params.cpp`

```cpp
// 外参随机游走噪声（慢变参数）
double g_ext_rot_noise  = 1e-4;  // rad/√s — 外参旋转噪声
double g_ext_trans_noise = 1e-4; // m/√s   — 外参平移噪声
double g_dt_noise        = 1e-5; // s/√s   — 时间偏移噪声
```

外参是**慢变参数**：温度、振动导致的缓慢漂移。随机游走噪声应设得很小。
或者在外参收敛后（后验方差 < 阈值）冻结其更新。

### 1.4 P 矩阵初始化

**文件**: `src/super_lio/src/lio/ESKF.cpp` — `SetInitialConditions()`

```cpp
// 外参初始不确定性
P_.block<3,3>(18, 18) = (1.0 * M_PI/180)² * I₃;  // 旋转: σ=1°
P_.block<3,3>(21, 21) = (0.05)² * I₃;             // 平移: σ=5cm
// LiDAR2 外参
P_.block<3,3>(24, 24) = (1.0 * M_PI/180)² * I₃;
P_.block<3,3>(27, 27) = (0.05)² * I₃;
// 时间偏移
P_(30, 30) = (0.01)²;  // δt1: σ=10ms
P_(31, 31) = (0.01)²;  // δt2: σ=10ms
```

### 1.5 可观测性分析

外参的某些维度可能不可观测：
- **纯旋转平台**（无平移运动）：平移外参不可观测
- **匀速直线运动**：LiDAR-IMU 平移外参的沿运动方向分量不可观测
- **单方向运动**：部分旋转外参不可观测

**解决方案**: 
- 在终端打印各外参维度的后验方差
- 方差 < 阈值的外参维度参与更新，否则跳过
- 论文中讨论可观测性条件

### 1.6 验证标准

```bash
# 测试1: 用 bag 回放，外参从错误初始值开始（偏差5°+5cm）
# 预期: 外参在30秒内收敛到真值附近（残差±1°, ±2cm）
# 验证: 对比在线标定结果 vs 手工测量结果

# 测试2: 用收敛后的外参替换初始值，定位精度提升
# 预期: APE 降低 10-20%
```

---

## Phase 2: Per-LiDAR Fisher 信息分析 (1-2 周)

### 目标
分离两个雷达的观测信息矩阵，计算各自的约束方向和互补性。

### 2.1 分离 Per-LiDAR HTVH

**文件**: `src/super_lio/src/lio/super_lio.cpp` — `Observe()`

当前做法：两个雷达的点一起进入 `Observe()`，累加到同一个 HTVH/HTVr。

**改为**: 分两次调用，分别累加：

```cpp
// 第一次: 只用 LiDAR1 的点
ObserveSingle(lidar1_mask, HTVH1, HTVr1, point_count1);
// 第二次: 只用 LiDAR2 的点  
ObserveSingle(lidar2_mask, HTVH2, HTVr2, point_count2);
// 合并: HTVH = HTVH1 + HTVH2, HTVr = HTVr1 + HTVr2
```

或者**更高效的做法**（避免两次 KNN）：在一次并行循环中，根据 intensity 符号判断雷达来源，分别累加到两个 ThreadACC：

```cpp
struct ThreadACC {
  M6d HTVH1 = M6d::Zero();  // LiDAR1
  V6d HTVr1 = V6d::Zero();
  M6d HTVH2 = M6d::Zero();  // LiDAR2
  V6d HTVr2 = V6d::Zero();
  int cnt1 = 0, cnt2 = 0;
};
```

### 2.2 Fisher 信息特征谱分析

**文件**: 新增 `src/super_lio/src/lio/fisher_analyzer.cpp`

```cpp
struct FisherInfo {
  V3d ev_H;           // HTVH[3:6,3:6] 特征值 (大→约束强)
  M3d evec_H;         // HTVH 特征向量
  double condition;   // 条件数 κ = λmax/λmin
  V3d constraint_dir; // 最强约束方向
  
  // 互补性度量
  static double complementarity(const FisherInfo& a, const FisherInfo& b) {
    // 两个雷达最强约束方向的夹角
    double dot = std::abs(a.constraint_dir.dot(b.constraint_dir));
    return 1.0 - dot;  // 0=平行(冗余), 1=正交(互补)
  }
};
```

### 2.3 互补性驱动的自适应权重

**文件**: `src/super_lio/src/lio/super_lio.cpp`

```cpp
// 在 Observe() 结尾，根据互补性调整下帧的 g_lidar2_weight
FisherInfo info1 = analyzeHTVH(HTVH1);
FisherInfo info2 = analyzeHTVH(HTVH2);

double comp = FisherInfo::complementarity(info1, info2);
double cond_ratio = info1.condition / (info2.condition + 1e-10);

// 互补性高 → LiDAR2 权重大；条件数相近 → 等权
g_lidar2_weight = 0.5 + 0.5 * comp;  // [0.5, 1.0]
// 低通滤波平滑
g_lidar2_weight = 0.95 * prev_weight + 0.05 * g_lidar2_weight;
```

### 2.4 验证标准

```bash
# 测试: 不同场景下观察互补性指标变化
# 长廊: 两个雷达都看墙面 → comp ≈ 0 (冗余) → 降低融合权重
# 楼梯: LiDAR1看前方, LiDAR2看斜上方 → comp ≈ 0.6 (互补) → 提高
# 室外: 互补性波动大
```

---

## Phase 3: 退化感知的自适应融合 (1-2 周)

### 目标
在 Phase 2 每雷达信息分析基础上，实现完整的三级退化感知和自适应切换。

### 3.1 三级退化分类

**文件**: `src/super_lio/include/lio/ESKF.h` — 扩展现有 `DegeneracyOptions`

```cpp
enum class DegeneracyLevel {
  NORMAL = 0,   // 6 DOF 充分约束
  PARTIAL = 1,  // 1-2 方向弱（长廊）
  SEVERE = 2    // 3+ 方向弱（开阔地/退化完全）
};

struct DegeneracyResult {
  DegeneracyLevel level;
  int degen_dims;           // 退化方向数
  V3d weak_directions[3];   // 退化方向向量
  double condition;         // κ 值
  double complementarity;   // 当前互补性
};
```

### 3.2 退化预测（在退化发生前感知）

**文件**: `src/super_lio/src/lio/super_lio.cpp`

```cpp
// 使用当前速度 + IMU 预测未来 2 秒轨迹
// 查询地图中未来位置的几何特征
// 如果预测到退化，提前降低 LiDAR 权重

SE3 future_pose = predictPose(kf_state, imu_history, 2.0); // 2秒后
auto future_geometry = ivox_->sampleGeometry(future_pose);
if (future_geometry.condition > g_degeneracy_htvh_cond_thresh * 0.7) {
  // 即将退化，提前准备
  LOG(WARNING) << "Degeneracy predicted in ~2s, κ_pred=" 
               << future_geometry.condition;
}
```

### 3.3 IMU 主导模式切换

**文件**: `src/super_lio/src/lio/ESKF.cpp`

```cpp
if (degen_result.level >= DegeneracyLevel::PARTIAL) {
  // 退化方向: 暂停 LiDAR 观测修正
  // 退化方向 dx 投影到 0（当前 RemapDegeneracy 逻辑）
  // 额外: 退化方向的 bg/ba 更新也被抑制
  
  // 非退化方向: 正常更新
  
  // IMU 过程噪声自适应:
  // 退化时降低 Q 以信任 IMU（因为无外部修正）
  if (degen_result.level == DegeneracyLevel::SEVERE) {
    adaptive_Q = 0.1 * Q_;  // 降低过程噪声 → 信任 IMU
  }
}
```

### 3.4 退化恢复策略

**文件**: `src/super_lio/src/lio/ESKF.cpp`

```cpp
// 离开退化区域后:
// 1. 非退化方向的协方差已正确收敛 → 立即恢复正常更新
// 2. 退化方向的协方差因 IMU 积分而膨胀 → 需要"软重定位"
// 3. LiDAR 观测重新出现时，给退化方向更大的步长（临时降低 P⁻¹）

if (degen_result.level == NORMAL && prev_level >= PARTIAL) {
  // 退化恢复: 暂时增大退化方向的观测步长
  for (int d : prev_degen_dims) {
    P_block(d, d) *= 4.0;  // 放大退化方向的不确定性
  }
  LOG(INFO) << "Degeneracy recovered, re-converging...";
}
```

### 3.5 验证标准

```bash
# 测试1: 长廊场景，观察终端日志
# 预期: "Degeneracy (κ): κ=350 > 50" 持续触发
#        "Level: PARTIAL, weak dir: [0.98, 0.02, -0.17]" (走廊方向)

# 测试2: 长廊 → 开阔地 过渡
# 预期: Level 从 PARTIAL → NORMAL 平滑过渡

# 测试3: APE 对比 (退化场景)
# 预期: 退化感知 ON vs OFF → APE 在长廊方向降低 50%+
```

---

## Phase 4: 结构化楼梯边缘约束 (2 周)

### 目标
提取楼梯场景的结构化边缘（平行水平线），构建等间距 + 平行性约束。

### 4.1 边缘特征聚类

**文件**: 新增 `src/super_lio/src/lio/edge_clustering.cpp`

在现有边缘特征点提取后，对边缘方向向量做聚类：

```cpp
struct EdgeCluster {
  V3 direction;           // 平均线方向
  std::vector<V3> points; // 属于该聚类的边缘点
  int count;
  double direction_variance;
};

// 聚类算法: 对 line_dir_vec_ 中有效的边缘做方向聚类
// 同方向（夹角 < 15°）且空间邻近 → 同一聚类
std::vector<EdgeCluster> clusterEdges(
  const std::vector<V3>& line_dirs,
  const std::vector<V3>& line_centers,
  const std::vector<char>& edge_mask,
  int edge_count
);
```

### 4.2 楼梯结构检测

```cpp
struct StairStructure {
  std::vector<EdgeCluster> step_edges;  // 台阶边缘线（水平、平行）
  V3 stair_direction;                   // 台阶法向（行走方向）
  double step_height;                   // 估算台阶高度
  double step_depth;                    // 估算台阶深度
  bool valid;
};

// 检测条件:
// 1. 至少 3 条平行水平线（方向与 gravity 正交）
// 2. 线间在 vertical 方向等间距（方差 < 阈值）
// 3. 线间在 stair_direction 方向等间距（方差 < 阈值）
StairStructure detectStairStructure(
  const std::vector<EdgeCluster>& clusters,
  const V3& gravity_dir
);
```

### 4.3 楼梯 Pseudo-Measurement

检测到楼梯结构后，添加两个 pseudo-measurement 残差：

**Pseudo-Measurement 1: 方向一致性约束**

多条台阶线必须平行，约束机器人旋转：

```
r_dir = Σ (clusters[i].direction × clusters[i+1].direction)  // 叉积 → 0
```

**Pseudo-Measurement 2: 等间距约束**

约束台阶法向上的位姿增量：

```
r_step = (step_height - prior_height) / σ_height
```

```cpp
// 在 Observe() 结尾，如果有楼梯结构，添加额外残差
if (stair.valid) {
  // 等间距: 约束 z 方向步长
  V6d J_step = computeStairJacobian(stair, kf_state);
  double r_step = stair.step_height - expected_step_height;
  double w_step = 500.0;  // 楼梯约束权重
  HTVH += J_step * w_step * J_step.transpose();
  HTVr -= J_step * w_step * r_step;
  
  LOG(INFO) << "Stair constraint: height=" << stair.step_height
            << "m, " << stair.step_edges.size() << " edges";
}
```

### 4.4 验证标准

```bash
# 测试: 多楼层爬楼梯 bag
# 预期: 
# - 终端打印 "Stair constraint" 日志（确认检测到楼梯）
# - z 漂移 < 0.1m（对比无楼梯约束的 > 0.3m z 漂移）
# - 楼梯段 APE 整体降低 30%+
```

---

## Phase 5: 实验评估 (3-4 周)

### 5.1 消融实验设计

在 B2 平台上录制 5 个场景的 bag（每个 3-5 分钟）：

| 场景 | 退化类型 | 验证重点 |
|------|---------|---------|
| 长廊 (100m) | 纵向退化 | Phase 3 IMU主导模式 |
| 楼梯 (3层) | z方向累计漂移 | Phase 4 结构边缘 |
| 室内混合 | 光照变化 | Phase 1 在线标定 |
| 室外空地 | 完全退化 | Phase 3 严重退化恢复 |
| 室内→室外过渡 | 场景剧变 | Phase 2 自适应权重 |

对每个场景运行 5 个配置：

| 配置 | 在线标定 | 自适应融合 | 退化恢复 | 楼梯约束 |
|------|---------|-----------|---------|---------|
| Baseline | ✗ | ✗ | ✗ | ✗ |
| +Calib | ✓ | ✗ | ✗ | ✗ |
| +Calib+Fusion | ✓ | ✓ | ✗ | ✗ |
| +Calib+Fusion+DegRec | ✓ | ✓ | ✓ | ✗ |
| Full (Ours) | ✓ | ✓ | ✓ | ✓ |

### 5.2 评价指标

```
- ATE (Absolute Trajectory Error): RMSE, mean, median, std
- RPE (Relative Pose Error): per 10m segment
- 退化段 APE：只统计长廊/楼梯段的 ATE
- z-drift rate: mm/s（楼梯场景特有）
- 收敛时间：外参收敛到 ±1°/±2cm 的时间
- 计算耗时：per-frame processing time (ms)
```

### 5.3 公开数据集对比

- M2DGR (多场景、多传感器)
- 或 HILTI SLAM Challenge (足式机器人)
- 对比方法: FAST-LIO2, Faster-LIO, DLIO

### 5.4 B2 数据集开源

- ROS 2 bag 格式
- 包含: /rslidar_points, /livox/lidar, /dog_imu_raw
- 可选: /joint_states, /foot_force
- Ground truth: 用高精度建图结果（或 motion capture）
- 上传到 GitHub + Zenodo (DOI)

---

## 实施时间线

```
Week  1-2:  Phase 1.1-1.3 (ESKF状态扩展 + 雅可比推导)
Week  3:    Phase 1.4-1.6 (P矩阵 + 可观测性 + 验证)
Week  4:    Phase 2 (per-LiDAR Fisher 分离 + 互补度量)
Week  5:    Phase 3.1-3.2 (三级退化分类 + 预测)
Week  6:    Phase 3.3-3.5 (IMU主导切换 + 恢复 + 验证)
Week  7:    Phase 4.1-4.2 (边缘聚类 + 楼梯检测)
Week  8:    Phase 4.3-4.4 (pseudo-measurement + 验证)
Week  9-10: Phase 5.1-5.2 (消融实验 + 数据采集)
Week 11:    Phase 5.3-5.4 (公开数据集 + 对比实验)
Week 12-14: 论文写作 + 修改
```

---

## 文件清单

### 修改现有文件

| 文件 | Phase | 改动内容 |
|------|-------|---------|
| `include/lio/ESKF.h` | 1,3 | 状态扩展、DegeneracyOptions 扩展、退化恢复接口 |
| `src/lio/ESKF.cpp` | 1,3 | 雅可比扩展、退化分类、IMU主导切换 |
| `include/lio/params.h` | 1,2,4 | 新参数声明（外参噪声、楼梯约束权重等） |
| `src/lio/params.cpp` | 1,2,4 | 新参数默认值 |
| `include/lio/super_lio.h` | 2,4 | per-LiDAR HTVH 成员、边缘聚类成员 |
| `src/lio/super_lio.cpp` | 2,4 | 分离 HTVH、互补度量、楼梯检测 |
| `src/ros/ROSWrapper.cpp` | 1,2,4 | 新参数加载、Fisher 日志发布 |
| `config/b2.yaml` | 1,2,4 | 新配置项 |

### 新增文件

| 文件 | Phase | 内容 |
|------|-------|------|
| `src/lio/fisher_analyzer.cpp` | 2 | Fisher 信息特征谱分析 |
| `include/lio/fisher_analyzer.h` | 2 | FisherInfo 结构体、互补度量 |
| `src/lio/edge_clustering.cpp` | 4 | 边缘聚类 |
| `include/lio/edge_clustering.h` | 4 | EdgeCluster 结构体 |
| `src/lio/stair_detector.cpp` | 4 | 楼梯结构检测 |
| `include/lio/stair_detector.h` | 4 | StairStructure、检测逻辑 |

---

## 关键决策点

### D1: 外参状态用 SE(3) 还是 6-DOF 向量？

**推荐**: SE(3) — 与现有 ESKF 的 SO(3) 一致，使用右扰动。雅可比在 Solà 2018 中有标准公式。

### D2: 是否用协方差交叉？

**推荐**: 先假设两个雷达观测独立（直接堆叠），在论文中讨论相关性来源，附录中推导 CI 公式但不实现。如果审稿人要求再加。

### D3: 退化预测用地图查询还是纯启发式？

**推荐**: 先用纯启发式（当前 HTVH 条件数趋势外推），论文中讨论"如果有预建地图可以做更准确预测"。这比实现完整的地图查询简单且效果类似。

### D4: 楼梯约束是硬约束还是软约束？

**推荐**: 软约束（pseudo-measurement with weight）。硬约束（固定台阶高度）在台阶高度不一致时会引入偏差。

---

## 论文大纲

```
1. Introduction
   - 足式机器人定位挑战: 长廊退化、楼梯z漂移、多LiDAR标定
   - 现有方法局限: 离线标定、无退化感知、忽略结构先验
   - Contributions: (1)(2)(3)

2. Related Work
   - LIO / ESKF-LIO
   - Multi-LiDAR systems
   - Degeneracy in SLAM
   - Structured features

3. Methodology
   3.1 System Overview
   3.2 Online Spatiotemporal Calibration
   3.3 Per-LiDAR Fisher Information Analysis
   3.4 Degeneracy-Aware Adaptive Fusion
   3.5 Structured Stair Constraints

4. Experiments
   4.1 Implementation Details
   4.2 Ablation Study
   4.3 Degeneracy Scenario Analysis
   4.4 Public Dataset Comparison
   4.5 Runtime Analysis

5. Conclusion
```

---

## 风险与缓解

| 风险 | 概率 | 缓解 |
|------|------|------|
| 外参不可观测导致漂移 | 中 | 可观测性检测 + 冻结方差大的维度 |
| 时间复杂度增加 >50% | 中 | 分离 HTVH 用单次循环 + 双Accumulator |
| 楼梯检测误触发 | 低 | 保守阈值 + 连续3帧确认 |
| 自适应权重振荡 | 低 | 低通滤波平滑 + 权重变化限幅 |
