# 面向退化环境的鲁棒多LiDAR定位 — 论文工作计划

## 工作清单

### Phase A: 系统完善 (2-3周)

| # | 任务 | 产出 | 验证标准 |
|---|------|------|---------|
| A1 | 在线标定收敛验证 | 外参收敛曲线图 | 从错误初值收敛到±1°、±3cm内 |
| A2 | 标定雅可比符号最终确认 | 代码修复 | 外参向正确方向更新(非反向) |
| A3 | 块对角数值稳定性分析 | 条件数前后对比 | LDLT不失败，κ<10^8 |
| A4 | 确保全模块可独立开关 | yaml配置 | 每个模块有enable开关 |
| A5 | 统一日志输出格式 | 终端输出 | 每帧摘要: 点数/退化状态/外参/残差 |

### Phase B: 数据采集 (2周)

| # | 任务 | 产出 | 验证标准 |
|---|------|------|---------|
| B1 | 长廊往返(100m+) | bag 1 | 含直线+拐角+小通道，预建地图 |
| B2 | 多楼层楼梯(3层+) | bag 2 | 上下楼梯，含z漂移验证 |
| B3 | 室内混合(办公室/大厅) | bag 3 | 光照变化、玻璃、动态物体 |
| B4 | 室外开放空间 | bag 4 | 少量特征，验证完全退化恢复 |
| B5 | 场景切换(室内→室外) | bag 5 | 传感器视野剧变 |
| B6 | 录制ground truth轨迹 | 真值文件 | 高精度建图或mocap |

### Phase C: 消融实验 (2周)

| # | 配置 | 退化检测 | P下界 | 退化恢复 | 标定 | 强度加权 | 边缘特征 |
|---|------|---------|------|---------|------|---------|---------|
| C1 | Baseline(原始Super-LIO) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| C2 | +Degeneracy | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| C3 | +P-Bound | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ |
| C4 | +Recovery | ✓ | ✓ | ✓ | ✗ | ✗ | ✗ |
| C5 | +Calib | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ |
| C6 | Full (Ours) | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

每个场景跑6个配置，共6×6=36组实验。

| # | 任务 | 产出 |
|---|------|------|
| C7 | 6场景×6配置 ATE/RPE 表格 | 主实验结果 |
| C8 | 退化段局部ATE对比(长廊/楼梯) | 退化专项分析 |
| C9 | 外参收敛曲线(错误初值→正确值) | 标定验证 |
| C10 | 时间序列图(单条轨迹可视化) | 定性展示 |
| C11 | 运行时分析(各模块耗时) | 实时性证明 |

### Phase D: 公开数据集对比 (1周)

| # | 任务 | 配置 |
|---|------|------|
| D1 | M2DGR或HILTI数据集上跑Ours | Full配置 |
| D2 | 同数据集跑FAST-LIO2 | 官方代码+适配 |
| D3 | 同数据集跑DLIO | 官方代码+适配 |
| D4 | ATE/RPE对比表格 | 3方法×N序列 |

### Phase E: 论文写作 (3-4周)

| # | 章节 | 内容要点 |
|---|------|---------|
| E1 | Introduction | 足式机器人定位挑战、长廊/楼梯退化、贡献三点 |
| E2 | Related Work | LIO、Multi-LiDAR、Degeneracy in SLAM、Robust Estimation |
| E3.1 | System Overview | 系统框图、传感器配置、坐标系定义 |
| E3.2 | Multi-LiDAR ESKF-LIO | 双雷达点云处理、IEKF扩展、联合观测模型 |
| E3.3 | Degeneracy Detection | 后验σ + HTVH κ双指标、退化解映射 |
| E3.4 | Degeneracy Recovery | 恢复状态机、P下界动态调整、bg/ba冻结、地图冲突检测 |
| E3.5 | Online Calibration | SE(3)在线标定、块对角求解、长度归一化(可选:放appendix) |
| E4.1 | Implementation Details | 硬件配置、参数设置 |
| E4.2 | Ablation Study | C1-C6消融表 |
| E4.3 | Degeneracy Analysis | 退化段专项、恢复时间统计 |
| E4.4 | Public Dataset | D1-D4对比 |
| E4.5 | Runtime | 各模块耗时 |
| E5 | Conclusion | 总结+future work |

---

## 知识清单

### 数学基础

| 知识点 | 用途 | 参考文献 |
|--------|------|---------|
| **SE(3)/SO(3)李群李代数** | 理解ESKF误差状态、右扰动、雅可比推导 | Solà 2018 "A micro Lie theory for state estimation" |
| **ESKF (Error-State KF)** | 整个系统的数学框架，Predict/Update/P推导 | Solà 2017 "Quaternion kinematics for ESKF" |
| **IEKF迭代重线性化** | UpdateObserve中4-6次迭代的收敛性 | Bell & Cathey 1993 "The iterated Kalman filter update" |
| **SMW恒等式(Sherman-Morrison-Woodbury)** | P^{-1}+H^TWH求逆化简 | Golub & Van Loan "Matrix Computations" Ch.2 |
| **Fisher信息矩阵** | HTVH作为观测信息的几何解释 | Cramér-Rao bound相关文献 |
| **协方差交叉(Covariance Intersection)** | 双雷达信息融合时可能的理论支撑(可选) | Julier & Uhlmann 1997 "A non-divergent estimation algorithm" |
| **PCA/特征值分解** | 线特征提取、退化方向分析 | Jolliffe "Principal Component Analysis" |
| **条件数分析** | HTVH κ作为退化指标 | Trefethen & Bau "Numerical Linear Algebra" |
| **卡方分布/保护级** | 可选:残差一致性检验、完整性监控 | RTCA DO-229 (GNSS integrity) |

### 算法基础

| 知识点 | 用途 | 参考文献 |
|--------|------|---------|
| **点-面ICP残差推导** | Super-LIO核心观测模型 | Low 2004 "Linear Least-Squares Optimization for Point-to-Plane ICP" |
| **点-线ICP残差推导** | 边缘特征约束 | LOAM系列 (Zhang & Singh 2014) |
| **OctVoxMap/体素哈希** | 地图数据结构、KNN搜索 | Behley & Stachniss 2018 "Surfel-based mapping" |
| **退化检测(Zhang 2016)** | 解映射理论 | Zhang et al. 2016 "On degeneracy of optimization-based state estimation problems" |
| **NDT配准** | 重定位模块 | Biber & Straßer 2003 "NDT scan registration" |
| **ScanContext** | 可选:回环检测(未实现) | Kim & Kim 2018 "Scan Context" |

### 系统知识

| 知识点 | 用途 | 参考文献 |
|--------|------|---------|
| **FAST-LIO2架构** | Super-LIO基础框架，理解对比 | Xu et al. 2022 "FAST-LIO2: Fast Direct LiDAR-inertial Odometry" |
| **DLIO** | 对比方法 | Chen et al. 2023 "DLIO: Direct LiDAR-Inertial Odometry" |
| **多LiDAR标定** | 在线标定相关工作 | Lv et al. 2021 "Targetless Calibration of LiDAR-IMU System" |
| **Livox Mid-360特性** | LiDAR2的非重复扫描、FOV | Livox官方文档 |
| **RoboSense-32特性** | LiDAR1的360°扫描 | RoboSense官方文档 |

### 实验技能

| 知识点 | 用途 |
|--------|------|
| **evo工具** | ATE/RPE计算、轨迹可视化 |
| **ROS2 bag录制/回放** | 数据集录制 |
| **PCL点云处理** | 数据预处理、可视化 |
| **rviz2可视化** | 定性debug |
| **LaTeX/IEEE模板** | RA-L/ICRA投稿格式 |

---

## 论文投稿建议

| 目标 | 期刊/会议 | 页数限制 | 特点 |
|------|----------|---------|------|
| 首选 | **RA-L + ICRA 2027** | RA-L 8页 + ICRA abstract | RA-L期刊+顶会presentation |
| 备选 | **IEEE T-RO** | 20页 | 需要更完整理论和实验 |
| 快速 | **IROS 2027** | 6页 | 时效快但影响力略低 |

RA-L+ICRA是最佳路径: RA-L审稿周期2-3月→接收→ICRA做oral/poster。
