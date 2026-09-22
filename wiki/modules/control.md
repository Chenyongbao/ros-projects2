# Control 纯追踪控制

> 源码：`src/robot/control/`（`control_node.cpp` + `control_core.cpp`）

## 职责

订阅 Planner 的 `/plan` 路径和里程计 `/odom`，输出底盘速度指令 `/cmd_vel`（`geometry_msgs/Twist`：线速度 + 角速度）。

## Pure Pursuit 纯追踪

**Pure Pursuit**（音标 /pjʊə ˈpɜːsjuːt/：Pure Pursuit 纯追踪：模拟人类司机"看向远处一点再开过去"的几何跟踪算法，把转向控制转化为追踪一个前瞻点）的核心思想：不在原地盯紧路径，而是**追踪路径前方一个前瞻点**，像司机看远不看近。

### 前瞻点选取 `findLookaheadPoint`

沿路径找**第一个距离 ≥ `lookahead_dist_`** 的路径点，且有两个约束：

- 只考虑车身前方的点：方位角与车头夹角 > 90°（π/2）的点在后方，跳过以防倒车/折返
- 剩余所有点都在前瞻距离内（接近终点）时，取路径最后一个点

### 转向计算 `computeCommand`

1. **到达判定**：与路径终点距离 < `goal_tolerance_` → 返回零速指令
2. 计算机器人指向前瞻点的方位角 `angle_to_target = atan2(dy, dx)`
3. **航向误差** = 方位角 − 当前 yaw，并归一化到 [−π, π]，保证沿最短方向旋转

```cpp
while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
```

4. **原地旋转行为**：|误差| > `max_steering_angle_` 时线速度置零，先原地转向对齐车头，避免大**转弯半径**（Turning radius 转弯半径：差分小车以固定角速度画圆的半径，速度越快半径越大）冲出路径
5. **P 控制转向**：`angular.z = Kp * heading_error`（Kp = 2.0），即**比例控制**（音标 /ˈprəʊpɔːʃənl kənˈtrəʊl/：P Control 比例控制：输出与误差成正比的经典反馈控制，简单稳定但有静差）

## 从四元数提取航向角

与 Map Memory 相同的标准公式：

```cpp
robot_yaw_ = atan2(2.0*(q.w*q.z + q.x*q.y),
                   1.0 - 2.0*(q.y*q.y + q.z*q.z));
```

## 参数

见 `config/params.yaml`：`linear_speed_`（巡航线速度）、`lookahead_dist_`（前瞻距离）、`goal_tolerance_`（终点容差）、`max_steering_angle_`（最大转向误差阈值）。

调参直觉：

| 现象 | 调整 |
|---|---|
| 转弯切内/切外明显 | 增大 `lookahead_dist_`（更平滑）或减小（更贴线） |
| 高频左右摆动（**振荡**（音标 /ˌɒsɪˈleɪʃn/：Oscillation 振荡：控制系统输出来回摆动不收敛，通常是 Kp 过大导致）） | 减小 Kp |
| 弯道冲出路径 | 减小 `linear_speed_` 或收紧原地旋转触发阈值 |
