# 第四章 在线时空联合标定算法

§3 建立了外参失配导致往返非对称偏移的因果模型：$\delta\boldsymbol{\xi}^*_{\text{drift}} = \mathbf{H}_{\text{pose}}^{-1} \cdot \Delta\mathbf{C} \cdot \boldsymbol{\xi}_\Delta$。本章基于该模型提出 32 维增广 ESKF 在线时空标定算法，估计外参相对失配并驱动 $\boldsymbol{\xi}_\Delta \to \mathbf{0}$，从成因上消除往返非对称偏移。

## 4.1 32 维增广 ESKF 状态空间

### 4.1.1 状态向量定义

系统状态以标准 18 维 IMU 导航误差状态为基础：

$$\delta\mathbf{x}_{\text{nav}} = [\delta\boldsymbol{\theta}^\top, \delta\mathbf{p}^\top, \delta\mathbf{v}^\top, \delta\mathbf{b}_g^\top, \delta\mathbf{b}_a^\top, \delta\mathbf{g}^\top]^\top \in \mathbb{R}^{18}$$

其中 $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ 为 SO(3) 右扰动旋转误差，$\delta\mathbf{p}, \delta\mathbf{v}$ 分别为位置与速度误差，$\delta\mathbf{b}_g, \delta\mathbf{b}_a$ 为陀螺与加速度计零偏误差，$\delta\mathbf{g}$ 为重力向量误差。所有导航误差状态默认定义在 IMU 坐标系，无需额外标注帧下标。

在此基础上，引入双雷达的外参误差和时偏，构成 14 维标定误差状态：

$$\delta\mathbf{x}_{\text{calib}} = [\boldsymbol{\xi}_1^\top, \boldsymbol{\xi}_2^\top, \delta t_1, \delta t_2]^\top \in \mathbb{R}^{14}$$

其中外参误差 $\boldsymbol{\xi}_k = [\boldsymbol{\theta}_k^\top, \boldsymbol{\rho}_k^\top]^\top \in \mathbb{R}^6$，与式 (1) 中定义一致。时偏 $\delta t_k \in \mathbb{R}$ 为雷达 $L_k$ 相对于 IMU 的残差。

综上，全量增广状态为导航误差状态与标定误差状态的拼接：

$$\delta\mathbf{x} = \delta\mathbf{x}_{\text{nav}} \oplus \delta\mathbf{x}_{\text{calib}} \in \mathbb{R}^{32}$$

其中 $\oplus$ 为列向量拼接操作。各分量的内部索引布局见附录 A。

### 4.1.2 预测模型

名义状态的更新采用 SE(3) 右扰动：

$$\begin{aligned}
\mathbf{R} &\leftarrow \mathbf{R} \cdot \exp(\Delta\boldsymbol{\theta}) \\
\mathbf{t} &\leftarrow \mathbf{t} + \mathbf{R} \cdot \Delta\mathbf{p} \\
\hat{\mathbf{T}}_k &\leftarrow \hat{\mathbf{T}}_k \cdot \exp(\Delta\boldsymbol{\xi}_k), \quad k=1,2 \\
\delta t_k &\leftarrow \delta t_k + \Delta\delta t_k
\end{aligned}$$

标定误差状态与导航误差状态存在本质差异：导航误差状态经 IMU 测量驱动运动学方程传播，标定误差状态（外参与时偏）则无运动学模型可循。若将标定误差状态建模为恒定值（$\dot{\boldsymbol{\xi}}_k = \mathbf{0}$），其协方差矩阵 $\mathbf{P}_{\text{calib}}$ 将在持续观测下坍缩至零，卡尔曼增益随之趋零，滤波器永久丧失修正能力；而温度、振动等物理因素会使外参与时偏发生真实漂移，届时 §3 的非对称偏移将重新出现。因此，将标定误差状态建模为零均值、无回归项的连续时间维纳过程：

$$\begin{aligned}
\dot{\boldsymbol{\xi}}_k(t) &= \mathbf{w}_{\text{ext},k}(t), \quad \mathbf{w}_{\text{ext},k} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_{\text{ext},k}) \\[4pt]
\dot{\delta t}_k(t) &= w_{t,k}(t), \quad w_{t,k} \sim \mathcal{N}(0, \sigma_{t,k}^2)
\end{aligned}$$

其中 $\mathbf{w}_{\text{ext},k} \in \mathbb{R}^6$ 与 $w_{t,k} \in \mathbb{R}$ 为连续时间高斯白噪声，功率谱密度分别为 $\boldsymbol{\Sigma}_{\text{ext},k}$ 与 $\sigma_{t,k}^2$。该模型无记忆，不假设偏移向任何先验值回归，与永久性物理偏移一致。噪声结构取各向同性且旋转-平移解耦：

$$\boldsymbol{\Sigma}_{\text{ext},k} = \text{diag}\Big(\sigma_{\text{ext,rot}}^2 \mathbf{I}_{3\times 3}, \; \sigma_{\text{ext,trans}}^2 \mathbf{I}_{3\times 3}\Big) \in \mathbb{R}^{6\times 6}$$

在采样间隔 $\Delta t$ 下，维纳过程增量 $\Delta W = \int_t^{t+\Delta t} w(\tau)\,\mathrm{d}\tau$ 服从 $\mathcal{N}(0, \sigma^2 \Delta t)$，故协方差递推为 $P \leftarrow P + \sigma^2 \Delta t$。预测步不改变名义标定值，仅对协方差对角线执行严格对角注入，即不人为制造传感器间或维度间的耦合：

$$\begin{aligned}
P_{(18+i, 18+i)} &\mathrel{+}= \sigma_{\text{ext,rot}}^2 \cdot \Delta t, \quad i \in \{0,1,2,6,7,8\} \\[2pt]
P_{(21+i, 21+i)} &\mathrel{+}= \sigma_{\text{ext,trans}}^2 \cdot \Delta t, \quad i \in \{0,1,2,6,7,8\} \\[2pt]
P_{(30+i, 30+i)} &\mathrel{+}= \sigma_{dt}^2 \cdot \Delta t, \quad i \in \{0,1\}
\end{aligned}$$

对角注入符合物理独立原则，因为两雷达的温度形变与振动响应为独立随机过程，传感器间的协方差耦合交由观测更新建立。导航误差状态由标准 IMU 误差状态动力学驱动，其与标定误差状态的交叉协方差块在预测步保持不变。

综上，标定预测子模型为：

$$\begin{aligned}
\text{Nominal state:} &\quad \hat{\mathbf{T}}_k \leftarrow \hat{\mathbf{T}}_k, \quad \delta t_k \leftarrow \delta t_k \\[4pt]
\text{Covariance:} &\quad \mathbf{P}_{(18:31, 18:31)} \leftarrow \mathbf{P}_{(18:31, 18:31)} + \Delta t \cdot \text{diag}\big(\sigma_{\text{rot}}^2\mathbf{I}_3, \sigma_{\text{trans}}^2\mathbf{I}_3, \sigma_{\text{rot}}^2\mathbf{I}_3, \sigma_{\text{trans}}^2\mathbf{I}_3, \sigma_{dt}^2, \sigma_{dt}^2\big) \\[4pt]
\text{Cross-covariance:} &\quad \mathbf{P}_{(0:17, 18:31)} \leftarrow \mathbf{P}_{(0:17, 18:31)}
\end{aligned}$$

该模型遵循的原则是，预测步只增长不确定性，不改变名义标定值，标定的修正完全留给观测更新。

---

### 4.1.3 预测模型详细推导

> 本节为 §4.1.2 中预测模型的设计依据、离散化步骤和参数选择提供完整推导，供审稿参考；论文正文可直接引用 §4.1.2 的结论而不含本节细节。

---

首先考察恒定模型的可行性。若设 $\sigma_{\text{ext}} = 0$（$\dot{\boldsymbol{\xi}}_k = \mathbf{0}$），外参被建模为未知常量。在此模型下，ESKF 的预测步不对标定协方差注入任何过程噪声，$\mathbf{P}_{\text{calib}}$ 在每帧观测更新后单调递减。随着观测累积（$\sim 10^4$ 帧），$\mathbf{P}_{\text{calib}} \to \mathbf{0}$，Kalman gain $\mathbf{K} = \mathbf{P}\mathbf{H}^\top(\mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R})^{-1} \to \mathbf{0}$，滤波器由此永久拒绝后续的标定修正。

然而，以下物理因素会导致外参在运行中发生真实的缓慢偏移：

1. 温度形变使铝制支架产生热膨胀，其系数约为 $2.3 \times 10^{-5} / ^\circ\text{C}$，50 cm 支架在 20°C 温差下产生约 0.2 mm 的平移变化
2. 振动松动使足式机器人在着地冲击（$\sim 3\text{--}5g$）下长时间运行后螺丝预紧力衰减，外参发生不可逆偏移
3. 晶振温漂使低成本 IMU 时钟源产生可达 $\pm 50$ ppm 的频率偏移，1 小时累积约 180 ms 的时偏

恒定模型无法适应这些变化，因为一旦外参漂离初始估计，滤波器便因协方差坍缩而无法修正，§3 的非对称偏移将重新出现。

---

为论证维纳过程的选择，比较两种备选随机过程模型：

| | 维纳过程 | 一阶 Gauss-Markov |
|:---|:---|:---|
| SDE | $\dot{x} = w(t)$ | $\dot{x} = -\frac{1}{\tau}x + w(t)$ |
| 条件均值 | $\mathbb{E}[x(t)|x(0)] = x(0)$ | $\mathbb{E}[x(t)|x(0)] = x(0)e^{-t/\tau}$ |
| 稳态方差 | 发散（$\propto t$） | 有限（$= \frac{\tau}{2}\sigma^2$） |
| 物理含义 | 偏移无记忆，不回弹 | 偏移趋于衰减回零 |

Gauss-Markov 的回归项 $-\frac{1}{\tau}x$ 隐含一个先验："外参倾向于回到初始标定值"。但这在标定场景中是错误的，例如机器人撞墙导致 LiDAR 偏移了 $2^\circ$ 时，该偏移在物理上是永久的，不应被拉回。因此维纳过程的无记忆性更符合物理事实。

注：维纳过程方差发散（$\propto t$）在理论上是问题，但在算法中被观测更新持续压缩，因此后验协方差始终有限（其机理见下文的随机游走与可观性分析）。

---

维纳过程 SDE 的离散化如下。$\dot{x} = w(t)$ 在区间 $[t, t+\Delta t]$ 上的解为：

$$x(t+\Delta t) = x(t) + \int_t^{t+\Delta t} w(\tau) d\tau$$

$\int_t^{t+\Delta t} w(\tau) d\tau$ 是维纳过程的增量 $\Delta W$，由白噪声的积分性质：

$$\Delta W \sim \mathcal{N}(0, \sigma^2 \Delta t)$$

因此离散时间协方差的递推为 $\text{Var}[x(t+\Delta t)] = \text{Var}[x(t)] + \sigma^2 \Delta t$。代码中并不数值积分该 SDE，而是直接使用其二阶矩性质，在 $\mathbf{P}$ 的对角线上加上 $\sigma^2 \Delta t$。这是维纳过程在 Kalman 滤波中的标准处理方式。

对于外参 6 维向量 $\boldsymbol{\xi}_k$：旋转分量 $[\boldsymbol{\theta}_k]$ 对应方差增量 $\sigma_{\text{ext,rot}}^2 \Delta t \cdot \mathbf{I}_3$（$\text{rad}^2$），平移分量 $[\boldsymbol{\rho}_k]$ 对应 $\sigma_{\text{ext,trans}}^2 \Delta t \cdot \mathbf{I}_3$（$\text{m}^2$）。时偏标量 $\delta t_k$ 对应 $\sigma_{dt}^2 \Delta t$（$\text{s}^2$）。

---

对角注入的合理性由以下三条依据支撑。标定随机游走噪声严格注入协方差对角线，不人为制造传感器间或维度间的 off-diagonal 耦合：

1. 物理独立原则。LiDAR1 与 LiDAR2 是两个物理上独立的传感器，安装在机器人不同位置，其温度形变、振动响应、时钟偏移是独立随机过程。人为注入 ext1↔ext2 协方差等价于假设"一个雷达的外参漂了，另一个也会跟着漂"，这在物理上错误。

2. 耦合应由观测建立。ext1↔ext2 之间的协方差耦合应在 Phase 2 观测更新中自然产生：当两个雷达共享几何约束（例如同时观测同一墙面）时，$\mathbf{H}^\top\mathbf{V}\mathbf{H}|_{\text{calib}}$ 的 off-diagonal 块通过 Kalman gain 在 $\mathbf{P}_{\text{calib}}$ 中建立传感器间相关性，这种"观测驱动的耦合"比"先验注入的耦合"更可信。

3. 与 $\mathbf{F}_X$ 的一致性。状态转移矩阵 $\mathbf{F}_X$ 的标定块为 $\mathbf{I}_{14}$（右下 14×14 子块），其 $\mathbf{0}_{18\times14}$ 块表达了"IMU 运动不传播到标定"；对角注入的随机游走噪声则表达了"标定自身缓慢偏移，但偏移方向不可预测"。两者共同构成完整的标定预测子模型，其中 $\mathbf{F}_X$ 定义了确定性的"不动"部分，$\sigma^2\Delta t$ 提供了随机性的"偏移"部分。

---

噪声参数的取值由其物理含义决定，总结如下：

| 参数 | 单位 | 物理含义 | 典型范围 |
|:---|:---|:---|:---|
| $\sigma_{\text{ext,rot}}$ | $\text{rad}/\sqrt{\text{s}}$ | 每秒外参旋转随机游走标准差 | $10^{-5} \sim 10^{-4}$ |
| $\sigma_{\text{ext,trans}}$ | $\text{m}/\sqrt{\text{s}}$ | 每秒外参平移随机游走标准差 | $10^{-5} \sim 10^{-4}$ |
| $\sigma_{dt}$ | $\sqrt{\text{s}}$ | 每秒时偏随机游走标准差 | $10^{-6} \sim 10^{-5}$ |

参数选取面临双向约束：若 $\sigma$ 过小，$\mathbf{P}_{\text{calib}}$ 将坍缩至零，Kalman gain $\to 0$，外参永久冻结；若 $\sigma$ 过大，$\mathbf{P}_{\text{calib}}$ 增长过快，Kalman gain 保持较大，每帧观测大幅修正标定，估计被观测噪声污染，外参在真值附近震荡。

经验上，$\sigma$ 应设置为使系统在无观测的 10 秒内，标定不确定性增长至初始不确定性的 $\sim 10\%$。例如外参旋转初始 $\sigma_0 = 3^\circ \approx 0.05$ rad，目标 $\sigma_{10} \approx 0.005$ rad，则 $\sigma_{\text{ext,rot}} = 0.005/\sqrt{10} \approx 1.6 \times 10^{-3}$ rad/√s。

---

最后考察随机游走与可观性的交互。标定的后验协方差由预测步的噪声注入和观测步的信息增益共同决定：

$$\mathbf{P}_{\text{calib, post}} = \left(\mathbf{P}_{\text{calib, prior}}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}|_{\text{calib}}\right)^{-1}$$

- 预测步中，随机游走使 $\mathbf{P}_{\text{calib, prior}}$ 上升，持续注入不确定性；
- 观测步中，$\mathbf{H}^\top\mathbf{V}\mathbf{H}|_{\text{calib}}$ 提供信息，将不确定性压缩

两者的平衡点决定了稳态后验协方差。在退化方向（如走廊 pitch），$\mathbf{H}^\top\mathbf{V}\mathbf{H}|_{\text{calib}}$ 在该方向的信息增量极小，此时若 $\sigma = 0$，后验协方差会坍缩；反之若 $\sigma > 0$，随机游走持续注入不确定性，将后验协方差维持在合理水平。这意味着滤波器在遭遇差异化激励（台阶、坡道）时仍有充分的 Kalman gain 去修正，因为随机游走防止了退化方向协方差的过早坍缩；pitch 收敛慢但不发散的特性由第五章实验验证。

---

## 4.2 时空联合雅可比矩阵

### 4.2.1 时间畸变与三类雅可比

LiDAR $L_k$ 在时间戳 $t_i$ 捕获点 $\mathbf{p}_i^{\text{raw}}$，其实际 IMU 采样时刻为 $t_i + \delta t_k$。在运动状态下，点在 IMU 系的实际坐标一阶近似为：

$$\mathbf{p}_i^{\text{imu}}(t_i + \delta t_k) \approx \hat{\mathbf{R}}_k \mathbf{p}_i^{\text{raw}} + \hat{\mathbf{t}}_k + \mathbf{v}_i^{\text{imu}} \delta t_k$$

其中 $\mathbf{v}_i^{\text{imu}} = \mathbf{R}^\top \mathbf{v}_I + (\boldsymbol{\omega}_I)^\wedge (\hat{\mathbf{R}}_k \mathbf{p}_i^{\text{raw}} + \hat{\mathbf{t}}_k)$ 为点在 IMU 系下的等效线速度，$\mathbf{v}_i^{\text{imu}} \delta t_k$ 为时间偏差造成的运动畸变补偿项。

将上述投影代入点-面残差 $r_i = \mathbf{n}_i^\top (\mathbf{R} \mathbf{p}_i^{\text{imu}} + \mathbf{t} - \mathbf{q}_i)$，并引入 IMU 系法向量 $\mathbf{n}_i^{\text{imu}} = \mathbf{R}^\top \mathbf{n}_i$，通过 SE(3) 右扰动链式法则得到三类雅可比。位姿雅可比与 §3 式 (6) 一致：

$$\mathbf{J}_{\text{pose},i} = \begin{bmatrix} -(\mathbf{n}_i^{\text{imu}})^\top (\mathbf{p}_i^{\text{imu}})^\wedge, & (\mathbf{n}_i^{\text{imu}})^\top \end{bmatrix} \in \mathbb{R}^{1 \times 6}$$

外参雅可比在 $\hat{\mathbf{T}}_k$ 处取值，与 §3 式 (5) 对齐，其旋转分量含距离归一化 $1/s_0$：

$$\mathbf{J}_{\text{ext},k,i} = \begin{bmatrix} -\frac{1}{s_0}(\mathbf{n}_i^{\text{imu}})^\top \hat{\mathbf{R}}_k (\mathbf{p}_i^{\text{raw}})^\wedge, & (\mathbf{n}_i^{\text{imu}})^\top \hat{\mathbf{R}}_k \end{bmatrix} \in \mathbb{R}^{1 \times 6}$$

时间偏移雅可比由畸变补偿项 $\mathbf{v}_i^{\text{imu}} \delta t_k$ 对 $\delta t_k$ 求导得到：

$$J_{t_k,i} = \frac{1}{s_0} (\mathbf{n}_i^{\text{imu}})^\top \mathbf{v}_i^{\text{imu}} \in \mathbb{R}$$

其中 $s_0 = \text{median}(\{\|\mathbf{p}_i^{\text{raw}}\|\})$ 为帧内中值点距离，作为距离归一化因子将旋转雅可比的距离量级统一至 $O(1)$。

### 4.2.2 稀疏性与累积

观测空间为 20 维，由位姿（6 维）、双雷达外参（各 6 维）与时偏（2 维）构成。在此观测空间内，单点观测雅可比 $\mathbf{J}_i \in \mathbb{R}^{20}$ 仅有 13 个非零分量，即位姿 6 维、该点所属雷达的外参 6 维与时偏 1 维；另一雷达对应的 7 维严格为零，因为该点不在其视场内。

每帧所有有效特征点以加权最小二乘形式累积为观测信息矩阵与残差向量，其中点 $i$ 的权重 $w_i$ 由传感器源、反射强度与特征类型三个因子相乘构成：

$$\mathbf{H}^\top\mathbf{V}\mathbf{H} = \sum_i w_i \mathbf{J}_i \mathbf{J}_i^\top \in \mathbb{R}^{20 \times 20}, \quad \mathbf{H}^\top\mathbf{V}\mathbf{r} = -\sum_i w_i \mathbf{J}_i r_i \in \mathbb{R}^{20}$$

其中残差过大的点由二值拒斥条件剔除，即满足 $|r_i| \geq \sqrt{l_i}/9$（$l_i = \|\mathbf{p}_i^{\text{raw}}\|$ 为点距，门限随点距自适应放宽）的点不参与上述累积。

---

## 4.3 两步软解耦 IEKF 求解器

### 4.3.1 动机与两步法概述

多传感器系统的观测残差由位姿误差与标定误差（外参与时偏）两类来源叠加而成。位姿误差为动态量，随运动与场景变化剧烈，对残差的贡献可达米级；标定误差为静态量，仅随温度、振动等因素缓慢漂移，对残差的贡献为毫米至亚毫米级。二者在数量级上天然相差三至四个数量级。若将全部观测在单次线性求解中联合处理，量级较小的标定分量将被位姿残差淹没，导致标定收敛缓慢甚至发散。两步法的设计依据来自观测系统的三个结构特征：其一，观测雅可比具有稀疏性，每个残差仅与一个雷达的外参块相关联，两个外参块在观测维度上不相交；其二，上述量级差异使迭代初期自然由位姿修正主导；其三，导航误差状态经 IMU 运动学积分形成密集的内部耦合，而导航误差与标定误差之间的协方差耦合远弱于导航子空间内部的耦合。

据此，求解分两步进行。第一步（Phase 1）以导航优先方式求解位姿，标定观测暂不入阵，标定误差状态仅经先验协方差的非对角块获得软跟随修正；待位姿收敛后，第二步（Phase 2）在隔离了位姿噪声的 14 维标定子空间中对标定误差状态进行精修。该策略将全 32 维联合求解分解为规模分别为 18×18 与 14×14 的两个线性系统，前者保证位姿估计的实时收敛，后者在无位姿噪声干扰的条件下恢复标定的可辨识性。需要指出，该分解为软解耦而非完全解耦。完全解耦意味着两子系统在求解与信息层面均相互独立，在此并不可行，因为导航子空间内部的位置-速度-零偏-重力耦合密集，位姿残差须经导航子空间内的协方差传播方可间接修正这些维度。因此本方法对 14 维标定子空间执行舒尔补边缘化，将其协方差信息精确吸收进导航子空间后求解 18×18 导航系统，数学上等价于全 32 维求解而计算量更低；Phase 2 再于导航收敛后求解 14×14 标定子系统，其先验协方差携带 Phase 1 传播的耦合信息。两阶段仅在求解层面相互隔离，信息层面零损失。

### 4.3.2 求解过程

单帧观测的求解分为两步，分别对应导航优先的位姿估计与标定误差的后验修正。

**第一步（Phase 1）：导航优先的位姿估计。** 首先将先验协方差经旋转重参数化投影至当前切空间：

$$\mathbf{J}_\theta = \mathbf{I}_3 - \frac{1}{2}(\hat{\boldsymbol{\theta}})^\wedge, \quad \mathbf{P} \leftarrow \mathbf{J}_\theta \mathbf{P} \mathbf{J}_\theta^\top$$

随后在 20 维观测空间内累积观测信息矩阵与残差向量。对先验协方差按导航与标定子空间分块，并对标定子空间执行舒尔补边缘化，得到导航条件协方差：

$$\bar{\mathbf{P}}_{\text{nav}} = \mathbf{P}_{nn} - \mathbf{P}_{nc}\mathbf{P}_{cc}^{-1}\mathbf{P}_{cn} \in \mathbb{R}^{18 \times 18}$$

其中 $\mathbf{P}_{nn}, \mathbf{P}_{cc}$ 分别为导航与标定子块协方差，$\mathbf{P}_{nc}, \mathbf{P}_{cn}$ 为交叉协方差。将位姿观测信息叠加至边缘化导航信息矩阵，构造仅含位姿残差的右端项，求解 18×18 导航系统：

$$\mathbf{S}_{\text{nav}} = \bar{\mathbf{P}}_{\text{nav}}^{-1}, \quad \mathbf{S}_{\text{nav}}^{(0:5,0:5)} \mathrel{+}= \mathbf{H}^\top\mathbf{V}\mathbf{H}|_{(0:5, 0:5)}, \quad \mathbf{b}_{\text{nav}} = [(\mathbf{H}^\top\mathbf{V}\mathbf{r}|_{\text{pose}})^\top, \mathbf{0}_{12}^\top]^\top$$

$$\Delta\mathbf{x}_{\text{nav}} = \mathbf{S}_{\text{nav}}^{-1} \cdot \mathbf{b}_{\text{nav}}, \quad \mathbf{R} \leftarrow \mathbf{R} \cdot \exp(\Delta\boldsymbol{\theta}), \quad \mathbf{t} \leftarrow \mathbf{t} + \mathbf{R} \cdot \Delta\mathbf{p}$$

重复迭代直至修正量收敛。该 18×18 求解经舒尔补边缘化精确保留导航-标定耦合，与全 32×32 求解数学等价，而计算量由 $O(32^3)$ 降至 $O(14^3 + 18^3)$。此阶段标定观测不入阵，标定误差状态仅经协方差非对角块获得间接修正。

**第二步（Phase 2）：标定后验修正。** 位姿收敛后，提取 14 维标定子块，将标定观测信息叠加至标定先验信息之上：

$$\mathbf{S}_{\text{calib}} = \mathbf{P}|_{(18:31, 18:31)}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}|_{(6:19, 6:19)}, \quad \mathbf{b}_{\text{calib}} = \mathbf{H}^\top\mathbf{V}\mathbf{r}|_{(6:19)}$$

通过稠密 LDL^T 分解求解标定修正量并应用之，其中外参旋转分量经 $s_0$ 逆归一化恢复物理量纲：

$$\Delta\mathbf{x}_{\text{calib}} = \mathbf{S}_{\text{calib}}^{-1} \cdot \mathbf{b}_{\text{calib}}$$

最后更新后验协方差。导航子空间按信息形式更新整个 18×18 块，其中位姿观测信息以 $\mathbf{M}_{\text{pose}}$ 填充（仅在位姿 6×6 子块非零）；标定子块按信息形式更新为 $\mathbf{S}_{\text{calib}}^{-1}$：

$$\mathbf{P}_{(0:17, 0:17)} \leftarrow \big(\mathbf{P}_{(0:17, 0:17)}^{-1} + \mathbf{M}_{\text{pose}}\big)^{-1}, \quad \mathbf{P}_{(18:31, 18:31)} \leftarrow \mathbf{S}_{\text{calib}}^{-1}$$

其中 $\mathbf{M}_{\text{pose}}$ 为 $\mathbf{H}^\top\mathbf{V}\mathbf{H}|_{(0:5,0:5)}$ 在 18×18 导航块上的零填充，该更新在精确算术下与 Joseph 形式 $(I - \mathbf{K}\mathbf{H})\mathbf{P}$ 等价。交叉协方差块随上述更新经矩阵乘法同步传播，使 Phase 2 的标定先验携带 Phase 1 的信息。

上述各步的设计依据与数值论证详见 §4.3.3。

---

**退化场景中的可观测性分离**

长走廊中 ICP 位姿 Hessian $\mathbf{H}_{\text{pose}}$ 的纵向特征值 $\lambda_{\min} \to 0$。在此条件下，外参标定表现出反直觉的可观测性分离：

1. 绝对公共偏置不可观。若 $\boldsymbol{\xi}_1 = \boldsymbol{\xi}_2 \neq \mathbf{0}$（两雷达存在相同的系统误差），该公共误差将完美隐藏在位姿纵向滑移中，因为沿走廊方向的 $\delta\mathbf{p}_y$ 与 $\boldsymbol{\xi}_{1,2}$ 的平移 $y$ 分量的耦合矩阵在 $\mathbf{H}^{-1}$ 的退化方向上退化，滤波器无法独立辨识
2. 相对失配高度可观。若 $\boldsymbol{\xi}_1 \neq \boldsymbol{\xi}_2$，前后点云将产生内部几何撕裂，此时 $\Delta\mathbf{C} \neq \mathbf{0}$（§3.3.2），ESKF 的后验协方差限制发散边界，同时 $\mathbf{H}^\top\mathbf{V}\mathbf{r}|_{\text{calib}}$ 中的梯度驱动 Phase 2 优先消除相对差异

由此可得核心洞察：退化场景中在线标定的目标不是获取每个传感器的绝对外参真值，而是消除传感器间的相对时空不一致。这与 §3 式 (20) 的因果模型一致，即偏移量 $\delta\boldsymbol{\xi}^*_{\text{drift}} \propto \|\boldsymbol{\xi}_\Delta\|$，因此仅需驱动 $\boldsymbol{\xi}_\Delta \to \mathbf{0}$ 即可消除非对称偏移，而无需 $\boldsymbol{\xi}_k \to \mathbf{0}$。

---

### 4.3.3 两步法详细推导

> 本节为 §4.3.2 中各步骤提供完整的设计依据、数值论证和经验教训，供审稿参考；论文正文可直接引用 §4.3.1–4.3.2 的结论而不含本节细节。

---

**Step 0：为什么需要 J_θ 旋转重参数化**

$\mathbf{P}$ 存储的是上次更新时刻切空间中的协方差。本次观测发生时，名义状态已由 IMU 预测驱动发生了变化（$\mathbf{R}$ 被更新为 $\mathbf{R} \cdot \exp(\boldsymbol{\omega}\Delta t)$），新旧两个时刻的切空间不再重合。

对于 SO(3) 右扰动模型，若名义旋转从 $\hat{\mathbf{R}}_0$ 变为 $\hat{\mathbf{R}}_1 = \hat{\mathbf{R}}_0 \cdot \exp(\hat{\boldsymbol{\theta}})$，则在旧切空间中定义的误差状态 $\delta\boldsymbol{\theta}^{(0)}$ 与新切空间中的 $\delta\boldsymbol{\theta}^{(1)}$ 满足：

$$\delta\boldsymbol{\theta}^{(1)} \approx \left(\mathbf{I}_3 - \frac{1}{2}(\hat{\boldsymbol{\theta}})^\wedge\right) \delta\boldsymbol{\theta}^{(0)}$$

$\mathbf{J}_\theta = \mathbf{I}_3 - \frac{1}{2}(\hat{\boldsymbol{\theta}})^\wedge$ 正是切空间之间的变换矩阵。跳过这一步直接求解会导致 Kalman gain 的旋转分量出现 $O(\|\Delta\boldsymbol{\theta}\|)$ 的系统偏差，且该偏差在每帧迭代中被累积放大约 $\sim 5$ 倍（迭代次数）。

投影仅作用于前 30 维（索引 0–29，对应旋转/位置/速度/零偏/重力/外参 10 个三维块），跳过第 30–31 维标量时偏，因为时偏定义在欧氏空间 $\mathbb{R}$ 上，不受 SO(3) 切空间变化影响。

---

**Step 1：为什么在 20 维而非 32 维累积观测**

观测雅可比 $\mathbf{J}_i$ 在 32 维状态空间中仅有 13 个非零分量（位姿 6 + 一个雷达的外参 6 + 时偏 1），另一个雷达的 7 维严格为零。在 20 维观测空间累积 $\mathbf{H}^\top\mathbf{V}\mathbf{H}$ 和 $\mathbf{H}^\top\mathbf{V}\mathbf{r}$ 保持了这种稀疏性，避免了在 32 维空间中进行低效的零填充矩阵运算。升维操作推迟到求解时通过选择性的块填充完成。

---

**Step 2–3：为什么 Phase 1 不入标定观测信息**

这是两步法的核心设计选择，基于残差量级差异的定量分析：

| 残差来源 | 典型量级 | 物理含义 |
|:---|:---|:---|
| 位姿残差 $r_{\text{pose}}$ | $10^{-1} \sim 10^{1}$ m | 激光点与地图平面的投影距离 |
| 外参残差 $r_{\text{ext}}$ | $10^{-4} \sim 10^{-2}$ m | 外参偏差导致的点云内部几何不一致 |
| 时偏残差 $r_{dt}$ | $10^{-4} \sim 10^{-3}$ m | 时间偏差导致的运动畸变残差 |

若标定残差与位姿残差同时进入 RHS，线性系统 $\mathbf{A}\Delta\mathbf{x} = \mathbf{b}$ 中，标定分量对应的 RHS 条目量级仅为位姿的 $10^{-4}$ 倍。求解得到的 $\Delta\boldsymbol{\xi}_k$ 量级 $< 10^{-3}$，将被位姿修正（量级 $10^{-1}$）完全淹没，等价于浪费一次 18×18 求逆去计算无意义的微小修正。

不入标定观测的等价解释：告知 IEKF"标定观测暂时不可信，先用位姿观测把状态拉到大致正确的位置"。这避免了标定噪声对位姿收敛的干扰。

---

**Step 4：为什么导航子系统不可降阶为 6×6（经验教训）**

既然 RHS 仅 6 维非零，一个直观的想法是将 Phase 1 降阶为纯 6×6 位姿求解，从而省去 18×18 求逆。实际上这样做会导致轨迹发散，原因如下：

$\mathbf{P}^{-1}$ 在导航子空间（18 维）内部是**稠密**的。IMU 预测步的 $\mathbf{F}_X$ 在导航变量间建立了密集的 off-diagonal 耦合：

- $\mathbf{P}_{(3:5, 6:8)} \neq \mathbf{0}$（位置 ↔ 速度，来自 $\mathbf{F}_X^{(3:5,6:8)} = \mathbf{I}\Delta t$）
- $\mathbf{P}_{(3:5, 9:11)} \neq \mathbf{0}$（位置 ↔ 陀螺零偏，来自链式传播）
- $\mathbf{P}_{(6:8, 12:14)} \neq \mathbf{0}$（速度 ↔ 加计零偏）

当 18×18 系统求解时（标定子空间已经舒尔补边缘化），即使 RHS 仅位姿残差（6D）非零，$\mathbf{S}_{\text{nav}}^{-1}$ 也会将这些残差通过 off-diagonal 块传播至全部 18 维，使速度、零偏、重力均获得间接修正。若降阶为 6×6 求解，这些维度将完全丧失观测更新，仅在预测步被 IMU 噪声驱动，收敛性随之崩坏。

该结论已经由实际实验验证：代码曾尝试将 Phase 1 改为纯 6×6 逆，结果轨迹发散，恢复 18×18 边缘化求解后立即恢复稳定运行。

标定维度在 Phase 1 中不参与求解（$\Delta\mathbf{x}_{\text{calib}} = \mathbf{0}$），其信息经舒尔补边缘化并入导航先验，交叉协方差块在协方差更新中同步传播。

---

**Step 6：为什么位姿收敛后标定观测可以入阵**

位姿收敛条件 $\|\Delta\mathbf{x}\|_\infty < \epsilon$ 保证了名义残差 $r_i^0(\mathbf{T}^*)$ 已接近零。此时残差中剩余的误差主要来源于外参偏差 $\boldsymbol{\xi}_k$ 和时偏 $\delta t_k$，而非位姿误差。$\mathbf{b}_{\text{calib}}$ 中的信号几乎纯粹反映标定偏差，此时信噪比从 Phase 1 的 $\sim 10^{-4}$ 提升到 $\sim 10^{0}$，标定修正具备了物理意义。

---

**Step 7：LDL^T vs Cholesky，稠密 vs 块对角**

选择 LDL^T 的原因在于，$\mathbf{S}_{\text{calib}} = \mathbf{P}_{\text{calib}}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}|_{\text{calib}}$ 由两项叠加而成，且 $\mathbf{P}_{\text{calib}}$ 经随机游走预测协方差累积后，在退化方向（如平地场景的 pitch）条件数升高。标准 Cholesky 分解 $\mathbf{A} = \mathbf{L}\mathbf{L}^\top$ 需求 $\mathbf{A}$ 严格正定且在近奇异方向上涉及开根号 $\sqrt{a_{ii} - \sum l_{ik}^2}$，条件数高时精度损失显著。LDL^T 分解 $\mathbf{A} = \mathbf{L}\mathbf{D}\mathbf{L}^\top$ 避免开根号，对条件数较差的矩阵数值稳定性更好。

$\mathbf{S}_{\text{calib}}$ 的 ext1↔ext2 off-diagonal 块非零，因此必须稠密求解，其非零性来自两个来源：

1. $\mathbf{P}_{\text{calib}}^{-1}$ 中经多次预测-更新循环累积的传感器间协方差，其初始值为零，但随机游走与 Phase 1 软跟随在多帧迭代后使 off-diagonal 逐渐稠密化
2. Phase 1 的协方差更新 $(I - \mathbf{K}\mathbf{H})\mathbf{P}$ 过程中，交叉协方差块将位姿观测信息传播至标定子空间

因此标定子系统不能拆分为两个独立的 7×7 求解，而应采用 14×14 稠密 LDL^T。

---

**Step 8：$s_0$ 逆归一化的推导**

外参旋转雅可比 $\mathbf{J}_{\text{ext,rot}}$ 的物理量级正比于点的观测距离 $d = \|\mathbf{p}_i^{\text{raw}}\|$（杠杆臂效应：$\mathbf{p}^{\text{raw}} \times \mathbf{n}$ 的范数 $\propto d$）。若不做归一化，远距离点的旋转雅可比分量可达近距离点的数十倍，从而在 Gauss-Newton 近似 Hessian 中完全主导旋转方向的优化。为此，所有旋转雅可比在累积时除以帧内中值点距离 $s_0 = \text{median}(\{\|\mathbf{p}_i^{\text{raw}}\|\})$，将距离量级统一至 $O(1)$。

归一化对线性系统求解的影响。设真实（物理）系统为 $\mathbf{H}_{\text{true}} \cdot \Delta\boldsymbol{\theta}_{\text{true}} = \mathbf{b}_{\text{true}}$。归一化后 $\mathbf{J}_{\text{norm}} = \mathbf{J}_{\text{true}} / s_0$，代入信息矩阵与残差向量的累积：

$$\mathbf{H}_{\text{norm}} = \sum w_i (\mathbf{J}_{\text{true}} / s_0)(\mathbf{J}_{\text{true}} / s_0)^\top = \mathbf{H}_{\text{true}} / s_0^2, \quad \mathbf{b}_{\text{norm}} = -\sum w_i (\mathbf{J}_{\text{true}} / s_0) r_i = \mathbf{b}_{\text{true}} / s_0$$

求解归一化系统：

$$\Delta\boldsymbol{\theta}_{\text{norm}} = \mathbf{H}_{\text{norm}}^{-1} \cdot \mathbf{b}_{\text{norm}} = s_0^2 \cdot \mathbf{H}_{\text{true}}^{-1} \cdot (\mathbf{b}_{\text{true}} / s_0) = s_0 \cdot \Delta\boldsymbol{\theta}_{\text{true}}$$

归一化的影响是使求解器返回的 $\Delta\boldsymbol{\theta}$ 变为真实物理值的 $s_0$ 倍：Hessian 被 $1/s_0^2$ 缩放而 RHS 仅被 $1/s_0$ 缩放，方差的逆效应使得解被放大 $s_0$。因此需要除以 $s_0$（乘以 $1/s_0$）恢复为物理弧度，再通过 $\exp(\cdot)$ 映射到 SE(3) 流形。

外参平移雅可比 $\mathbf{J}_{\text{ext,trans}} = \mathbf{n}^{\text{lidar}}$（单位向量）与点距离无关，无需归一化，也无需逆归一化。位姿雅可比未做 $s_0$ 归一化，其旋转修正由 $\mathbf{J}_{\text{pose}}$ 的交叉项（pose↔ext coupling）间接获得距离不变性。

代码实现（[ESKF.cpp:530-533](src/super_lio/src/lio/ESKF.cpp#L530-L533)）：
```cpp
if (calib_s0_ > 1.0) {
    scalar inv_s0 = scalar(1.0 / calib_s0_);
    dx_calib.segment<3>(0) *= inv_s0;  // ext1 rot ÷ s0
    dx_calib.segment<3>(6) *= inv_s0;  // ext2 rot ÷ s0
}
```

---

**Step 9：为什么用信息形式而非 Joseph 形式更新协方差**

IEKF 的后验协方差有两种等价形式：

- Joseph 形式，即 $\mathbf{P}_{\text{post}} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{\text{prior}}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top$；
- 信息形式，即 $\mathbf{P}_{\text{post}} = (\mathbf{P}_{\text{prior}}^{-1} + \mathbf{H}^\top\mathbf{W}\mathbf{H})^{-1}$

在 IEKF 框架下两者精确等价。信息形式的优势在于 $\mathbf{P}_{\text{prior}}^{-1} + \mathbf{H}^\top\mathbf{W}\mathbf{H}$ 本身就是更新后的信息矩阵，从而避免 Joseph 形式中 $\mathbf{K}\mathbf{H}$ 的 18×18 矩阵-矩阵乘法开销（$\sim 18^3$ 运算），并直接复用 Phase 1/2 中已计算好的求逆/分解结果。对于本系统的维度（18 和 14），节省的运算量约 $18^3 / 3 \approx 2$k FLOPs，在 200 Hz LiDAR 帧率下不可忽略。

---

## 4.4 Remark: 推广到 $N$ 雷达

### 4.4.1 状态扩维

对于搭载 $N$ 个传感器的系统，增广状态扩展为 $18 + 7N$ 维（每个雷达贡献 6 维外参 + 1 维时偏）：

$$\delta\mathbf{x} = [\delta\mathbf{x}_{\text{nav}}^\top, \boldsymbol{\xi}_1^\top, \dots, \boldsymbol{\xi}_N^\top, \delta t_1, \dots, \delta t_N]^\top \in \mathbb{R}^{18 + 7N}$$

### 4.4.2 两步法的 Scaling 性质

- Phase 1 中，导航子空间固定为 18×18，全部标定维度（$7N$）经舒尔补边缘化吸收，求逆标度恒为 $O(18^3)$，与雷达数量无关；新增雷达的标定维度通过边缘化耦合获得间接修正，这与双雷达时的软跟随机制一致。
- Phase 2 中，标定子块变为 $(6N + N) \times (6N + N) = 7N \times 7N$ 稠密系统，标度 $O(N^3)$：

| $N$ | 标定维度 | LDL^T 规模 | 相对开销 |
|:---:|:---:|:---|:---|
| 2 | 14 | $14^3 \approx 2.7$k | 1× |
| 4 | 28 | $28^3 \approx 22$k | 8× |
| 8 | 56 | $56^3 \approx 176$k | 65× |

对于 $N > 4$ 的大规模部署（如自动驾驶冗余阵列），Phase 1 与 Phase 2 的 $O(N^3)$ 增长均不可忽略。可采用的缓解策略包括：Phase 1 中仅保留前两雷达的标定维度参与 off-diagonal 耦合，其余 $N-2$ 个雷达的标定设为块对角近似；Phase 2 中采用 Group-LDL 将每个雷达的 7 维视为独立群组，仅保留群内二阶耦合。此近似等价于假设不同雷达的残余外参误差随机游走在物理上独立，这恰是 §4.1.2 中对角注入的设计意图。

### 4.4.3 与 §3 相对差异界限的呼应

§3.4 式 (25) 指出 $N$ 传感器系统的偏移上界由 $\max_{i,j} \|\boldsymbol{\xi}_i - \boldsymbol{\xi}_j\|$ 控制。Phase 2 的稠密标定求解正是这一理论洞察的算法实现，即通过联合求解所有传感器间的耦合，在线标定主动使所有外参对向相对差异最大的传感器对趋同，从而持续压低系统的偏移风险。

### 4.4.4 双雷达最优性

对向部署的双雷达（前后各一，360° 有效视场）是足式机器人退化场景重定位的最优经济配置。$N > 2$ 的边际收益体现在：一方面，传感器冗余使个别雷达失效时系统仍能降级运行；另一方面，多视角竖直法向量提供更充分的 pitch/roll 激励，从而提升 pitch 可观性。主要应用场景在自动驾驶的多冗余激光雷达阵列。

---

## 附录 A：状态索引布局

全量增广状态 $\delta\mathbf{x} \in \mathbb{R}^{32}$ 的内部索引布局如下：

| 索引 | 维度 | 符号 | 描述 |
|:---:|:---:|:---|:---|
| 0–2 | 3 | $\delta\boldsymbol{\theta}$ | 旋转误差（SO(3) 右扰动） |
| 3–5 | 3 | $\delta\mathbf{p}$ | 位置误差 |
| 6–8 | 3 | $\delta\mathbf{v}$ | 速度误差 |
| 9–11 | 3 | $\delta\mathbf{b}_g$ | 陀螺零偏误差 |
| 12–14 | 3 | $\delta\mathbf{b}_a$ | 加速度计零偏误差 |
| 15–17 | 3 | $\delta\mathbf{g}$ | 重力向量误差 |
| 18–23 | 6 | $\boldsymbol{\xi}_1$ | LiDAR1 外参误差 $[\boldsymbol{\theta}_1, \boldsymbol{\rho}_1]$ |
| 24–29 | 6 | $\boldsymbol{\xi}_2$ | LiDAR2 外参误差 $[\boldsymbol{\theta}_2, \boldsymbol{\rho}_2]$ |
| 30 | 1 | $\delta t_1$ | LiDAR1 时偏 |
| 31 | 1 | $\delta t_2$ | LiDAR2 时偏 |
