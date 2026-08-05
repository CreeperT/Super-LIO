# 32 维增广 ESKF 与两步解耦策略推导

> 对应源码: `ESKF.h`, `ESKF.cpp`, `super_lio.cpp`

## 1. 增广误差状态空间

### 1.1 状态向量定义

$$
\delta\mathbf{x} = 
\begin{bmatrix}
\delta\boldsymbol{\theta}_I \\ \delta\mathbf{p}_I \\ \delta\mathbf{v}_I \\
\delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{g}
\end{bmatrix}
\oplus
\begin{bmatrix}
\delta\boldsymbol{\xi}_{L_1} \\ \delta\boldsymbol{\xi}_{L_2}
\end{bmatrix}
\oplus
\begin{bmatrix} \delta t_1 \\ \delta t_2 \end{bmatrix}
\in \mathbb{R}^{32}
$$

其中 $\oplus$ 表示维度拼接，非黎曼运算。

| 块 | 索引 | 维度 | 符号 | 说明 |
|:---|:---:|:---:|:---|:---|
| Core (nav) | 0–17 | 18 | $\delta\mathbf{x}_{\text{nav}}$ | SO(3) 右扰动误差状态 |
| Extrinsic L1 | 18–23 | 6 | $\delta\boldsymbol{\xi}_{L_1}$ | $[\delta\boldsymbol{\theta}_{IL_1}, \delta\mathbf{p}_{IL_1}]$ |
| Extrinsic L2 | 24–29 | 6 | $\delta\boldsymbol{\xi}_{L_2}$ | $[\delta\boldsymbol{\theta}_{IL_2}, \delta\mathbf{p}_{IL_2}]$ |
| Time offset | 30–31 | 2 | $\delta t_1, \delta t_2$ | LiDAR1/2 时偏残差 |

### 1.2 流形更新律

**导航状态**（SO(3) 右扰动）：

$$
\mathbf{R} \leftarrow \mathbf{R} \cdot \text{Exp}(\delta\boldsymbol{\theta}), \quad
\mathbf{p} \leftarrow \mathbf{p} + \mathbf{R} \cdot \delta\mathbf{p}
$$

**外参状态**（SE(3) 右扰动）：

$$
\mathbf{T}_{IL_k} \leftarrow \mathbf{T}_{IL_k} \cdot \text{Exp}(\delta\boldsymbol{\xi}_{L_k}), \quad k \in \{1,2\}
$$

其中 $\text{Exp}(\boldsymbol{\xi}) = \begin{bmatrix} \text{Exp}(\boldsymbol{\theta}) & \mathbf{J}_l(\boldsymbol{\theta})\mathbf{t} \\ \mathbf{0} & 1 \end{bmatrix}$。

**时偏状态**（线性加法）：

$$
\delta t_k \leftarrow \delta t_k + \Delta\delta t_k
$$

### 1.3 预测模型

导航状态由 IMU 运动学驱动；外参与时偏建模为随机游走：

$$
\dot{\delta\boldsymbol{\xi}}_{L_k} = \mathbf{n}_{\text{ext},k}, \quad
\dot{\delta t}_k = n_{t,k}
$$

预测协方差中，随机游走噪声为：

$$
\begin{aligned}
P_{(18+i,18+i)} &\mathrel{+}= \sigma_{\text{ext,rot}}^2 \cdot \Delta t, \quad i \in \{0,1,2,6,7,8\} \\
P_{(21+i,21+i)} &\mathrel{+}= \sigma_{\text{ext,trans}}^2 \cdot \Delta t, \quad i \in \{0,1,2,6,7,8\} \\
P_{(30,30)} &\mathrel{+}= \sigma_{dt}^2 \cdot \Delta t \\
P_{(31,31)} &\mathrel{+}= \sigma_{dt}^2 \cdot \Delta t
\end{aligned}
$$

其中 $\sigma_{\text{ext,rot}}, \sigma_{\text{ext,trans}}, \sigma_{dt}$ 由 `b2.yaml` 中的 `lio.calib.ext_rot_rw`、`lio.calib.ext_trans_rw`、`lio.calib.dt_rw` 参数控制。

---

## 2. 时空联合观测模型

### 2.1 点到面（Point-to-Plane）残差

LiDAR $L_k$ 在时刻 $t_i$ 捕获点 $\mathbf{p}_i^{L_k}$，其在全局地图坐标系中的投影为：

$$
\mathbf{p}_i^G = \mathbf{R}_I^G(t_i + \delta t_k) \big( \mathbf{R}_{L_k}^I \mathbf{p}_i^{L_k} + \mathbf{p}_{L_k}^I \big) + \mathbf{p}_I^G(t_i + \delta t_k)
$$

点到面残差为投影点到地平面法向量 $\mathbf{n}_i$ 的距离：

$$
r_i = \mathbf{n}_i^\top (\mathbf{p}_i^G - \mathbf{q}_i^G)
$$

### 2.2 雅可比分解

对增广状态 $\delta\mathbf{x}$ 的观测雅可比为 **20 维列向量**：

$$
\mathbf{J}_{\text{obs}} = \begin{bmatrix}
\underbrace{\mathbf{J}_{\text{pose}}}_{6} & \underbrace{\mathbf{J}_{\text{ext},k}}_{6} & \underbrace{\mathbf{J}_{\text{ext},k'}}_{6} & \underbrace{J_{t_1}}_{1} & \underbrace{J_{t_2}}_{1}
\end{bmatrix}^\top \in \mathbb{R}^{20}
$$

其中只有当前观测对应的雷达外参块非零（另一个雷达块保持为零）。

#### 2.2.1 位姿雅可比（6 维）

SO(3) 右扰动：

$$
\mathbf{J}_{\text{pose}} =
\begin{bmatrix}
-\mathbf{n}_i^\top \mathbf{R}_I^G (\mathbf{p}_i^I)^\wedge, &
\mathbf{n}_i^\top \mathbf{R}_I^G
\end{bmatrix}
$$

其中 $\mathbf{p}_i^I = \mathbf{R}_{L_k}^I \mathbf{p}_i^{L_k} + \mathbf{p}_{L_k}^I$ 为点在 IMU 系下的坐标。

#### 2.2.2 外参空间雅可比（每个雷达 6 维）

对 $\delta\boldsymbol{\xi}_{L_k} = [\delta\boldsymbol{\theta}_{IL_k}, \delta\mathbf{p}_{IL_k}]$：

$$
\frac{\partial r_i}{\partial \delta\boldsymbol{\theta}_{IL_k}} =
-\mathbf{n}_i^\top \mathbf{R}_I^G \mathbf{R}_{L_k}^I (\mathbf{p}_i^{L_k})^\wedge
$$

$$
\frac{\partial r_i}{\partial \delta\mathbf{p}_{IL_k}} =
\mathbf{n}_i^\top \mathbf{R}_I^G
$$

距离归一化：实际计算中缩放因子 $s_0 = \|\mathbf{p}_i^{L_k}\|$，将旋转雅可比归一化：

$$
\tilde{\mathbf{J}}_{\text{ext,rot}} = \frac{1}{s_0} \cdot \mathbf{J}_{\text{ext,rot}}
$$

#### 2.2.3 时间偏移雅可比（每个雷达 1 维）

由机器人运动导致的时偏几何畸变：

$$
J_{t_k} = \frac{\partial r_i}{\partial \delta t_k} =
\mathbf{n}_i^\top \Big(
\mathbf{v}_I^G + \mathbf{R}_I^G (\boldsymbol{\omega}_I^\wedge) \big( \mathbf{R}_{L_k}^I \mathbf{p}_i^{L_k} + \mathbf{p}_{L_k}^I \big)
\Big)
$$

物理意义：时间偏差导致的点云畸变正比于传感器线速度与角速度。同样用 $s_0$ 归一化。

### 2.3 累积观测方程

每帧所有特征点的残差累积为：

$$
\mathbf{H}^\top\mathbf{V}\mathbf{H} = \sum_i w_i \mathbf{J}_{\text{obs},i} \mathbf{J}_{\text{obs},i}^\top, \quad
\mathbf{H}^\top\mathbf{V}\mathbf{r} = -\sum_i w_i \mathbf{J}_{\text{obs},i} \cdot r_i
$$

其中 $w_i$ 为 Huber 鲁棒权重。

---

## 3. 两步解耦策略（Block-Diagonal IEKF）

### 3.1 块对角近似动机

全 32 维状态的信息矩阵 $\mathbf{P}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}$ 中，位姿块与外参块之间的耦合由 $\mathbf{J}_{\text{pose}} \cdot \mathbf{J}_{\text{ext}}^\top$ 决定。在初始协方差下（$P_{(18:), (18:)}$ 较大），该耦合项被协方差逆 $\mathbf{P}^{-1}$ 的块对角结构抑制。因此近似认为**位姿子问题与外参子问题可解耦**。

### 3.2 Phase 1: 位姿 IEKF（6 维）

首次迭代构建 32×32 增广系统：

$$
(\mathbf{P}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}) \Delta\mathbf{x} = \mathbf{H}^\top\mathbf{V}\mathbf{r}
$$

但**仅求解位姿前 6 维**（外参块在后续单独处理）：

$$
\Delta\mathbf{x}_{\text{pose}}^{(m)} = \mathbf{Q}_{\text{pose}} \cdot \mathbf{H}^\top\mathbf{V}\mathbf{r}\big|_{\text{pose}}
$$

其中 $\mathbf{Q}_{\text{pose}} = (\mathbf{P}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H})^{-1}$ 为全 32×32 求逆，但右手边仅填充位姿残差 $\mathbf{H}^\top\mathbf{V}\mathbf{r}\big|_{\text{pose}}$（6 维），外参残差暂不进右手边。

此迭代重复 $m=1,\dots,M$ 次直至 $\|\Delta\mathbf{x}^{(m)}\|_\infty < \epsilon$（通常 3–5 次收敛）。

### 3.3 Phase 2: 标定 LDLᵀ 独立求解（14 维）

位姿 IEKF 收敛后，从 32 维系统中**提取标定子块**（索引 18–31）：

**状态协方差子块**（14×14）：

$$
\mathbf{P}_{\text{calib}} = \mathbf{P}_{(18:31, 18:31)} \in \mathbb{R}^{14 \times 14}
$$

**观测信息子块**（在 20 维观测中的索引 6–19）：

$$
\mathbf{S}_{\text{calib}} = \mathbf{P}_{\text{calib}}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H}\big|_{(6:19, 6:19)}
$$

**残差子块**（在 20 维观测向量中的索引 6–19）：

$$
\mathbf{r}_{\text{calib}} = \mathbf{H}^\top\mathbf{V}\mathbf{r}\big|_{(6:19)}
$$

**LDLᵀ 分解求解**：

$$
\Delta\mathbf{x}_{\text{calib}} = \mathbf{S}_{\text{calib}}^{-1} \cdot \mathbf{r}_{\text{calib}}
$$

其中 $\Delta\mathbf{x}_{\text{calib}} \in \mathbb{R}^{14}$ 的内部布局为：

| 索引 | 内容 |
|:---:|:---|
| 0–2 | $\Delta\boldsymbol{\theta}_{IL_1}$ (3) |
| 3–5 | $\Delta\mathbf{p}_{IL_1}$ (3) |
| 6–8 | $\Delta\boldsymbol{\theta}_{IL_2}$ (3) |
| 9–11 | $\Delta\mathbf{p}_{IL_2}$ (3) |
| 12 | $\Delta\delta t_1$ (1) |
| 13 | $\Delta\delta t_2$ (1) |

### 3.4 两步法的数学合理性

两步法的合理性来源于以下信息论结构：

1. **雅可比结构稀疏**：每个残差项 $r_i$ 只涉及**一个**雷达的外参（$r_i$ 由 $L_1$ 或 $L_2$ 产生，不同时涉及两者）。因此外参雅可比 $\mathbf{J}_{\text{ext},1}$ 与 $\mathbf{J}_{\text{ext},2}$ 在观测维度上是正交的。

2. **协方差块对角性**：在标定初始化阶段，经预测传播后 $\mathbf{P}_{(0:5, 18:31)} \approx \mathbf{0}$——IMU 噪声不耦合导航与标定维度。因此 $\mathbf{P}^{-1}$ 在导航/标定交界处为零。

3. **位姿收敛优先**：IEKF 迭代中，位移残差 $\mathbf{H}^\top\mathbf{V}\mathbf{r}\big|_{\text{pose}}$ 远大于标定残差（位移通常是米级，标定残差是毫米/毫秒级），因此前几次迭代几乎完全由位姿主导。

4. **标定后验修正**：位姿收敛后，残差 $\mathbf{r}\big|_{\text{pose}} \approx 0$，此时标定残差成为主要梯度来源。独立 14×14 LDLᵀ 求解避免了位姿方向的高频噪声对标定估计的污染。

### 3.5 协方差投影（J_θ 重参数化）

每次 IEKF 迭代后，需用 Jacobian 重参数化投影协方差：

$$
\mathbf{J}_\theta = \mathbf{I}_3 - \frac{1}{2}(\hat{\boldsymbol{\theta}})^\wedge
$$

仅作用于 **3 维块**（索引 0–29，共 10 个块，不含末尾时偏标量）：

$$
\mathbf{P}_{(0, j:j+2)} \leftarrow \mathbf{J}_\theta \cdot \mathbf{P}_{(0, j:j+2)}, \quad j \in \{0,3,6,\dots,27\}
$$

---

## 4. 时空标定收敛性质

### 4.1 可观性分析

| 维度 | 激励源 | 收敛速度 |
|:---|:---|:---|
| 外参平移 | 平移运动 | 快速（<10 帧） |
| 外参旋转 (yaw) | 旋转运动 + 走廊几何 | 快速（~100 帧） |
| 外参旋转 (pitch) | 竖直方向激励 | **较慢**（~1000 帧, ~0.3° 残差） |
| 外参旋转 (roll) | 横向倾斜激励 | 同 pitch |
| 时偏 | 角速度 + 线速度 | 快速（~0.1ms 精度） |

### 4.2 pitch 残差的物理原因

在平坦走廊场景中，$L_k$ 的 pitch 方向旋转与 Z 轴平移的耦合矩阵 $\mathbf{J}_{\text{ext,rot}} \leftrightarrow \mathbf{J}_{\text{ext,trans}}$ 的列空间在竖直方向接近共线，导致局部信息矩阵条件数升高。该维度的收敛依赖于机器人经历坡度变化或跨越台阶时产生的差异化观测。这是**退化环境的固有性质**，并非算法缺陷。

---

## 5. 推广：$N$ 雷达增广状态空间

### 5.1 $N$ 雷达的状态维度

设传感器数量为 $N$（$N \ge 1$），增广误差状态为：

$$
\delta\mathbf{x} =
\underbrace{\begin{bmatrix}
\delta\boldsymbol{\theta}_I \\ \delta\mathbf{p}_I \\ \delta\mathbf{v}_I \\
\delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{g}
\end{bmatrix}}_{\text{core nav } \mathbb{R}^{18}}
\;\oplus\;
\underbrace{\begin{bmatrix}
\delta\boldsymbol{\xi}_{L_1} \\ \vdots \\ \delta\boldsymbol{\xi}_{L_N}
\end{bmatrix}}_{\text{extrinsic } \mathbb{R}^{6N}}
\;\oplus\;
\underbrace{\begin{bmatrix} \delta t_1 \\ \vdots \\ \delta t_N \end{bmatrix}}_{\text{time offset } \mathbb{R}^{N}}
\quad\in\quad \mathbb{R}^{18 + 7N}
$$

**索引布局**：

| 块 | 起始索引 | 跨度 | 内容 |
|:---|:---:|:---:|:---|
| Core nav | 0 | 18 | $\delta\boldsymbol{\theta}, \delta\mathbf{p}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a, \delta\mathbf{g}$ |
| Extrinsic $L_k$ | $18 + 6(k-1)$ | 6 | $[\delta\boldsymbol{\theta}_{IL_k}, \delta\mathbf{p}_{IL_k}]$ |
| Time offset $L_k$ | $18 + 6N + (k-1)$ | 1 | $\delta t_k$ |

> $N=1$：$18 + 7 = 25$ 维（单雷达）  
> $N=2$：$18 + 14 = 32$ 维（当前设计）  
> $N=3$：$18 + 21 = 39$ 维  
> $N=4$：$18 + 28 = 46$ 维

### 5.2 预测模型推广

导航状态由 IMU 运动学驱动（与 $N$ 无关）。各雷达外参与时偏建模为独立随机游走：

$$
\dot{\delta\boldsymbol{\xi}}_{L_k} = \mathbf{n}_{\text{ext},k}, \quad
\dot{\delta t}_k = n_{t,k}, \quad k = 1,\dots,N
$$

其中 $\mathbf{n}_{\text{ext},k} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_{\text{ext}})$，$n_{t,k} \sim \mathcal{N}(0, \sigma_{dt}^2)$。

预测协方差注入：

$$
\begin{aligned}
P_{(18+6(k-1)+i,\;18+6(k-1)+i)} &\mathrel{+}= \sigma_{\text{ext,rot}}^2 \cdot \Delta t, \quad i \in \{0,1,2\}, \; k=1,\dots,N \\
P_{(18+6(k-1)+3+i,\;18+6(k-1)+3+i)} &\mathrel{+}= \sigma_{\text{ext,trans}}^2 \cdot \Delta t, \quad i \in \{0,1,2\}, \; k=1,\dots,N \\
P_{(18+6N+(k-1),\;18+6N+(k-1))} &\mathrel{+}= \sigma_{dt}^2 \cdot \Delta t, \quad k=1,\dots,N
\end{aligned}
$$

噪声参数对所有雷达共享同一组值（$\sigma_{\text{ext,rot}}, \sigma_{\text{ext,trans}}, \sigma_{dt}$），物理上合理——雷达的标定参数漂移速率相同，差异仅在初始外参配置。

### 5.3 观测模型推广

每个特征残差只涉及**一个**雷达的外参。设残差 $r_i$ 来自雷达 $L_k$，其观测雅可比为：

$$
\mathbf{J}_{\text{obs},i} =
\begin{bmatrix}
\underbrace{\mathbf{J}_{\text{pose}}}_{6} &
\mathbf{0}_{6} \; \cdots \; \underbrace{\mathbf{J}_{\text{ext},k}}_{6} \; \cdots \; \mathbf{0}_{6} &
0 \; \cdots \; \underbrace{J_{t_k}}_{1} \; \cdots \; 0
\end{bmatrix}^\top
\in \mathbb{R}^{6 + 6N + N}
$$

其中：
- $\mathbf{J}_{\text{pose}}$ 为位姿雅可比（6D），所有雷达共享
- $\mathbf{J}_{\text{ext},k}$ 仅在第 $k$ 个外参块非零（6D）
- $J_{t_k}$ 仅在第 $k$ 个时偏位置非零（1D）
- 其余 $6(N-1)$ 个外参维度和 $(N-1)$ 个时偏维度恒为零

**关键性质**：外参雅可比在传感器维度上正交——$\mathbf{J}_{\text{ext},p} \cdot \mathbf{J}_{\text{ext},q}^\top = \mathbf{0}$ 当 $p \neq q$，因为每个残差只属于一个雷达。

### 5.4 两步解耦策略推广

#### Phase 1：位姿 IEKF（独立于 $N$）

全 $(18+7N)$ 维信息矩阵中，位姿 IEKF 的 6 维求解**与 $N$ 无关**：

$$
\Delta\mathbf{x}_{\text{pose}}^{(m)} = (\mathbf{P}^{-1} + \mathbf{H}^\top\mathbf{V}\mathbf{H})^{-1}_{(0:5,0:5)} \cdot (\mathbf{H}^\top\mathbf{V}\mathbf{r})_{(0:5)}
$$

$N$ 增加只扩大矩阵尺寸，不影响位姿子块的求解过程。这是两步解耦的核心优势。

#### Phase 2：标定块对角求解（$7N$ 维）

位姿收敛后，标定子块的结构为**块对角加全耦合**：

$$
\mathbf{S}_{\text{calib}} = \mathbf{P}_{\text{calib}}^{-1} + 
\underbrace{\begin{bmatrix}
\mathbf{B}_1 & & \\
& \ddots & \\
& & \mathbf{B}_N
\end{bmatrix}}_{\text{观测信息: 块对角}}
$$

其中每个 $\mathbf{B}_k \in \mathbb{R}^{7 \times 7}$ 来自雷达 $L_k$ 所有观测的累积。**观测信息天然块对角**——不同雷达的观测残差在标定维度上不产生交叉项。但 **$\mathbf{P}_{\text{calib}}^{-1}$ 不是块对角**——预测步的随机游走注入不产生外参之间的耦合，因此 $\mathbf{P}_{\text{calib}}$ 也是块对角的。

> **结论**：$\mathbf{S}_{\text{calib}}$ 本身是块对角的。标定求解可以**按雷达独立进行**：
>
> $$
> \Delta\mathbf{x}_{\text{calib},k} = \mathbf{S}_k^{-1} \cdot \mathbf{r}_{\text{calib},k}, \quad k = 1,\dots,N
> $$
>
> 每个子问题仅 7×7，$N$ 个 7×7 独立求解替代一个 $7N \times 7N$ 联合求解。计算复杂度从 $O((7N)^3)$ 降为 $O(N \cdot 7^3) = O(N)$。

**物理直觉**：雷达之间的外参不直接相关——标定 $L_1$ 的外参不需要知道 $L_2$ 的观测。两步解耦自然保留了这种独立性。

### 5.5 J_θ 投影推广

J_θ 作用于所有基于 SO(3) 的 3 维旋转块（不含末尾的时偏标量）：

$$
\mathbf{P}_{(0, j:j+2)} \leftarrow \mathbf{J}_\theta \cdot \mathbf{P}_{(0, j:j+2)}, \quad
j \in \{0, 3, 6, \dots, 3 \times (5 + 2N)\}
$$

共 $6 + 2N$ 个 3-向量块（6 个 core + 2N 个外参旋转）。$N=2$ 时为 10 个块（当前实现）。

### 5.6 退化检测推广

退化检测仅考察 **HTVH 平移块**（$3 \times 3$），与外参维度无关：

$$
\kappa = \frac{\lambda_{\max}(\mathbf{H}^\top\mathbf{V}\mathbf{H}_{(3:5,3:5)})}
{\lambda_{\min}(\mathbf{H}^\top\mathbf{V}\mathbf{H}_{(3:5,3:5)})}, \quad
\sigma_{\text{trans}} = \sqrt{\frac{\text{tr}(\mathbf{P}_{(3:5,3:5)})}{3}}
$$

$\kappa$ 门槛 + $\lambda_{\min}$ 绝对阈值 + $\sigma_{\text{trans}}$ 后验检查——三者均与 $N$ 无关。$N$ 雷达时退化检测逻辑完全不变。

### 5.7 复杂度分析

| 操作 | $N=2$ (当前) | $N$ 雷达 | 缩放 |
|:---|:---:|:---|:---|
| 预测协方差注入 | 12 条对角更新 | $7N$ 条 | $O(N)$ |
| 位姿 IEKF（每次迭代） | 32×32 求逆 | $(18+7N) \times (18+7N)$ 求逆 | $O(N^3)$ |
| 标定求解 | 14×14 LDLᵀ | $N$ 个 7×7 LDLᵀ | $O(N)$ |
| J_θ 投影 | 10 个 3-向量块 | $6 + 2N$ 个块 | $O(N)$ |
| 观测构建 | 2 个雷达点云 | $N$ 个雷达点云 | $O(N)$ |

**位姿 IEKF 的全矩阵求逆是唯一超线性项**。但 $(18+7N)$ 在实际范围 $N \in [1,4]$ 内最大仅 46，远低于 Eigen 的固定→动态尺寸切换阈值。实际运行中，$N$ 从 2 增加到 4 的位姿求解耗时增加约 $(\frac{46}{32})^3 \approx 3\times$，但位姿求解仅占总耗时的 ~10%，整体影响 < 20%。

### 5.8 配置接口设计

每个 LiDAR 的配置为独立块：

```yaml
lio.lidars:
  - topic: "/rslidar_points"
    type: 2                 # HESAI16
    extrinsic: [x,y,z, R00..R22]
    blind: 1.0
    maxrange: 150.0
    filter_rate: 3
    time_offset: 0.0
    weight: 1.0

  - topic: "/livox/lidar"
    type: 8                 # LIVOX_PC2
    extrinsic: [x,y,z, R00..R22]
    blind: 1.0
    maxrange: 100.0
    filter_rate: 3
    time_offset: 36.15
    weight: 1.0
```

ROS 端自动订阅 `N` 个 topic，ESKF 自动扩展状态维度，观测构建自动分配外参块索引。所有算法组件无需修改即可适配 $N=1,5,10$。

### 5.9 数值概览

```
N 雷达增广状态 = 18(core) + 6N(ext) + N(dt) = 18 + 7N

                    N=1   N=2   N=3   N=4
总维度              25    32    39    46
标定子块维度         7    14    21    28
J_θ 投影块数         8    10    12    14
IEKF 矩阵尺寸      25²   32²   39²   46²
标定计算           O(1)  O(1)  O(1)  O(1)  (N × 7³, 线性)
```

**核心不变性质**：两步解耦在位姿求解中消去了 $N$ 的影响，标定求解中 $N$ 个 7×7 独立系统保持线性复杂度。系统的模块化程度与传感器数量解耦。