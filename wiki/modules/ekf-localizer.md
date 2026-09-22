# EKF 定位融合

> 源码：`src/robot/ekf_localizer/`（新增包）+ `sensor_simulator/`（新增包）

## 职责

把"抄仿真真值"的定位作弊器升级为**真实定位链路**：轮速里程计与 IMU 两路带噪传感器，经扩展卡尔曼滤波（EKF）融合出误差有界的位姿估计 `/odom/filtered`——下游 Planner / Control / Map Memory **接口零改动**。

## 数据流

```text
odometry_spoof（真值源）/odom_raw
    → sensor_simulator 按 Thrun 运动模型注噪
        → /wheel_odom（漂移位姿 + 协方差）
        → /imu/data（噪声角速度）
    → ekf_localizer 预测 + 更新 → /odom/filtered
```

## 传感器仿真 sensor_simulator

真实机器人只有带噪信息源。本节点模拟这一点：

- **Thrun 速度运动模型**（Probabilistic Robotics Ch.5）：

```text
σ²_v = α1·v² + α2·ω²    （线速度噪声）
σ²_ω = α3·v² + α4·ω²    （角速度噪声）
```

- 对真值速度 (v, ω) 采样高斯噪声后**欧拉积分**出漂移位姿——误差随距离无界累积，正是轮速计的真实特性
- 同步发布 IMU 角速度（独立采样 + σ_imu 方差），模拟陀螺仪短时精度高、漂移慢的特性

## EKF 滤波器 ekf_localizer

### 状态量

**x = [x, y, θ]ᵀ**（平面 3 自由度），协方差 P 为 3×3。

### 预测步（收到 wheel_odom 时）

差分底盘运动模型推状态，雅可比 F 传播协方差：

```text
x⁻ = [x + v·cosθ·dt,  y + v·sinθ·dt,  θ + ω·dt]
F = [[1, 0, -v·sinθ·dt],
     [0, 1,  v·cosθ·dt],
     [0, 0,  1         ]]
P⁻ = F·P·Fᵀ + Q        （Q 由 Thrun 模型的 σ² 经 dt² 映射，对角）
```

### 更新步一：位姿观测（wheel_odom）

z = [x, y, θ]，H = I₃，R 取轮速计协方差对角。完整 3×3 卡尔曼增益：

```text
S = P⁻ + R          （新息协方差）
K = P⁻·S⁻¹          （3×3 求逆，直接法）
x = x⁻ + K·(z - x⁻)  （角度维新息归一化到 [-π,π]）
P = (I - K)·P⁻
```

### 更新步二：IMU 偏航观测（imu/data）

IMU 只观测偏航角 → **单量（θ 维）KF 更新**：

```text
K = P_θθ / (P_θθ + r_imu)
θ' = θ + K·wrapAngle(yaw_meas - θ)
```

只收缩 θ 行列的协方差，不动 x/y——IMU 短时精度高，做高频纠偏；含 π/2 跳变保护。

### 收敛直觉

轮速计积分漂移时**协方差在增长**（预测步 Q 注入不确定性），观测一到就被拉回（更新步 K 加权纠偏）——不确定性越大观测权重越大，误差因此**有界**。这与纯积分（误差只增不减）的本质区别。

## 分层设计

`ekf_core.hpp/cpp` 是纯数学层（无 ROS 依赖）：`predict / updatePose / updateYaw / setMotionNoise`，可独立编译单测。节点层只做话题接线和参数。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| alpha1~alpha4 | 0.10/0.01/0.01/0.10 | Thrun 过程噪声系数（与 sensor_simulator 一致） |
| sigma_imu | 0.05 | IMU 偏航观测噪声标准差 (rad/s) |
| publish_rate | 10.0 | 融合位姿发布频率 (Hz) |

## 验证方式

Foxglove 中同时绘制 `/wheel_odom`（漂移轨迹）、`/odom/filtered`（融合轨迹）与真值对比：期望融合轨迹贴近真值、偏差有界，而纯轮速计轨迹随行驶距离持续发散。

## 简历表述

> 搭建真实定位链路：基于 Thrun 速度运动模型注入轮速计/IMU 噪声，实现扩展卡尔曼滤波位姿融合（3-DOF 状态、雅可比协方差传播、双观测源更新），里程计无界漂移收敛为有界误差，下游导航栈接口零改动。
