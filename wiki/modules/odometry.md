# Odometry Spoof 里程计

> 源码：`src/robot/odometry_spoof/`

## 职责

真实机器人上里程计（**odometry**（音标 /ɒˈdɒmɪtri/：Odometry 里程计：通过轮速计/IMU 积分估计机器人位姿的航迹推算技术，误差随时间累积））来自轮式编码器或融合定位。本工程运行在仿真中，`odometry_spoof` 直接从 Gazebo 仿真真值"转发"机器人位姿，封装成标准的 `nav_msgs/Odometry` 发布到 `/odom`。

- **Spoof（伪装/替代）**：不做积分推算，直接采用仿真器提供的 ground truth 位姿
- 好处：定位零误差，让导航栈专注验证**感知-规划-控制**链路本身
- 局限：没有体现真实世界中里程计漂移与噪声

## 消费方

| 节点 | 使用方式 |
|---|---|
| Planner | `updateOdometry(x, y)` 作为 A* 起点 |
| Control | 提取位置 + 四元数转 yaw，计算航向误差 |
| Map Memory | 提取位姿，做局部地图刚体变换；并按位移触发融合 |

## 扩展方向

若要在真实小车上复用本导航栈，只需将本节点替换为真正的定位来源（轮式里程计 + IMU 融合，或 AMCL 等定位包），下游接口不变 —— 这正是 node/core 分层带来的可替换性。
