# 技术方案：EKF 定位融合（第二阶段改造）

> 目标：干掉 odometry_spoof"抄仿真真值"的作弊器，建立**带噪传感器 → EKF 融合 → 有界误差定位**的真实定位链路。
> 这是真机必备能力，也是"仿真玩具"和"产品思维"的分水岭。

## 一、问题陈述

当前 `odometry_spoof` 直接从 Gazebo TF 抄真值位姿发布 `/odom/filtered`，全栈定位零误差。
真实机器人上只有两类带噪信息源：

- **轮速里程计**：积分型，短时平滑但误差随距离**无界累积**（打滑、轮径误差）
- **IMU（陀螺仪）**：角速度测量，短时精度高、漂移慢

EKF（扩展卡尔曼滤波）把两者融合：预测用运动模型推、更新用观测纠，输出误差有界的位姿估计。

## 二、总体数据流

```text
┌─────────────┐ /odom_raw（真值速度）  ┌──────────────────────┐
│ odometry_spoof│─────────────────────▶│ sensor_simulator(新) │
│  (改：只发速度) │                      │ Thrun 运动模型注噪    │
└─────────────┘                       │  ├─ /wheel_odom（噪声位姿+协方差）
                                      │  └─ /imu/data（噪声角速度+协方差）
                                      └──────────┬───────────┘
                                                 ▼
                                      ┌──────────────────────┐
                                      │ ekf_localizer(新)     │
                                      │ 预测(运动模型)          │
                                      │ 更新(两个观测源轮流纠)   │
                                      │ ──▶ /odom/filtered    │
                                      └──────────┬───────────┘
                                                 ▼
                              Planner / Control / Map Memory（接口零改动）
```

关键设计：**下游接口完全不变**。三个消费方（Planner/Control/Map Memory）仍订阅 `/odom/filtered`，只是数据来源从"真值"换成"EKF 估计"——这是 node/core 分层的红利。

## 三、状态量与模型

**状态 x = [x, y, θ]ᵀ**（平面 3 自由度，2D 差分底盘足够）

### 预测步（运动模型，收到 wheel_odom 时）

```text
x̂⁻ = f(x, u) = [x + v·cosθ·dt,  y + v·sinθ·dt,  θ + ω·dt]
```

- 控制量 u = (v, ω)：来自轮速里程计的 twist（已是噪声量测）
- 雅可比 F = ∂f/∂x：

```text
F = [ 1  0  -v·sinθ·dt ]
    [ 0  1   v·cosθ·dt ]
    [ 0  0   1         ]
```

- 过程噪声 Q 由 Thrun 速度运动模型（Probabilistic Robotics Ch.5）驱动：

```text
σ²_v = α1·v² + α2·ω²      σ²_ω = α3·v² + α4·ω²
Q = G · diag(σ²_v·dt², σ²_ω·dt²) · Gᵀ   （G 为噪声到状态的映射）
```

### 更新步（收到任一观测时）

标准 KF 更新：`K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹`，两路观测共用此框架：

| 观测源 | 观测量 z | H（观测矩阵） | R（量测噪声） |
|---|---|---|---|
| wheel_odom | 位姿 [x, y, θ] | I₃ | 对角阵，值随速度增大（注噪节点给出） |
| imu | 角速度 ω | [0, 0, 1]（量测 θ 变化率，实现为对 θ 的间接更新） | σ²_imu 常数 |

实现细节：IMU 只观测角速度，等价于在预测后用 `θ_meas ≈ θ + ω·dt` 做一次单量更新；工程上更简单的写法是把 IMU 当**高频 yaw 变化率观测**，只更新 θ 分量，不动 x/y。

### 协方差传播

```text
P⁻ = F·P·Fᵀ + Q        （预测）
P  = (I-KH)·P⁻          （更新）
```

## 四、sensor_simulator 节点（新）

**职责**：把真值变成"带噪传感器"，替代原 odometry_spoof 的输出角色。

- 订阅 `/odom_raw`（odometry_spoof 改发真值到此话题，含 twist）
- 按 **Thrun 速度运动模型**给 (v, ω) 加高斯噪声（α1~α4 参数化），欧拉积分出**漂移的噪声位姿**
- 发布：
  - `/wheel_odom`（`nav_msgs/Odometry`，噪声位姿 + 对角协方差）
  - `/imu/data`（`sensor_msgs/Imu`，噪声角速度 + 方差；角速度噪声独立采样）
- 10Hz 与真值同步（直接复用 odometry_spoof 的定时器节奏）

**真值注入方式**：`odometry_spoof` 仅改一行——把发布话题从 `/odom/filtered` 改为 `/odom_raw`，其余不动。它从"定位器"降级为"真值源"，这正是它该有的角色。

## 五、ekf_localizer 节点（新）

**职责**：核心滤波器。node + core 双层。

| 文件 | 内容 |
|---|---|
| `ekf_core.hpp/cpp` | 状态机纯数学：predict(u, dt)、updatePose(z, R)、updateYawRate(ω, R)、state()、covariance() —— **无 ROS 依赖可单测** |
| `ekf_node.hpp/cpp` | 订阅 `/wheel_odom`、`/imu/data`；wheel 回调触发 predict+updatePose，imu 回调触发 updateYawRate；10Hz 定时发布 `/odom/filtered`（位姿 + 融合协方差） |

参数（params.yaml）：

```yaml
alpha1: 0.10   # 线速度噪声随速度比例项
alpha2: 0.01   # 线速度噪声随角速度耦合项
alpha3: 0.01   # 角速度噪声随速度耦合项
alpha4: 0.10   # 角速度噪声随角速度比例项
sigma_imu: 0.05  # IMU 角速度量测噪声标准差 (rad/s)
publish_rate: 10.0
```

## 六、文件改动清单

| 文件 | 动作 |
|---|---|
| `src/robot/ekf_localizer/`（新包） | core（预测/更新数学）+ node（话题接线）+ params.yaml + CMakeLists + package.xml |
| `src/robot/sensor_simulator/`（新包） | 噪声注入节点（参考 taorobot odometry_noise_node，增加 IMU 通道） |
| `src/robot/odometry_spoof/src/odometry_spoof.cpp` | 发布话题 `/odom/filtered` → `/odom_raw`（一行） |
| `src/robot/bringup_robot/launch/robot.launch.py` | 挂载 sensor_simulator + ekf_localizer |

## 七、验证方式（不编译，静态核对 + 用户后续仿真）

- 静态：g++ 单独编译 ekf_core（纯数学无 ROS 依赖）做语法检查——与 path_smoother 同样的低风险验证
- 仿真（用户执行）：Foxglove 画 `/wheel_odom`（漂移轨迹）与 `/odom/filtered`（收敛轨迹）对比，期望 EKF 轨迹贴近真值、误差有界

## 八、简历表述（做完即可写）

> 搭建真实定位链路：基于 Thrun 速度运动模型注入轮速计/IMU 噪声，实现扩展卡尔曼滤波（EKF）位姿融合（3-DOF 状态、雅可比协方差传播、双观测源更新），里程计漂移误差从无界收敛至有界（仿真验证轨迹偏差 < X cm），下游导航栈接口零改动。
