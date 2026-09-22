# 仿真环境（Gazebo）

> 源码：`src/gazebo/launch/`

## 组成

| 文件 | 作用 |
|---|---|
| `robot_env.sdf` | 仿真世界：地面、墙体、障碍物等环境模型（**SDF**（音标 /ˌɛs diː ˈɛf/：SDF Simulation Description Format：Gazebo 的场景/模型描述格式，比 URDF 表达力更强）） |
| `robot.urdf` | 小车本体模型：差分驱动底盘 + 2D 激光雷达（**URDF**（音标 /ˈɜː dɑːf/：URDF Unified Robot Description Format：ROS 的机器人连杆/关节描述格式）） |
| `env.urdf` | 环境附加模型 |
| `sim.ign` | Ignition/Gazebo 启动配置：世界加载、插件、桥接（**bridge**（音标 /brɪdʒ/：Bridge ros_gz_bridge：把 Gazebo 话题与 ROS 2 话题互相转发的桥梁组件）） |
| `sim.launch.py` | ROS 2 launch 入口，一键拉起整个仿真 |

## 传感器与控制接口

- 激光雷达插件发布 `sensor_msgs/LaserScan`（经 bridge 转为 ROS 2 话题 `/scan`）
- 差分驱动插件订阅 `geometry_msgs/Twist`（`/cmd_vel`），按线/角速度驱动底盘
- 仿真真值位姿供 odometry_spoof 提取

## 数据链路

```text
Gazebo ──/scan──▶ Costmap ──局部地图──▶ Map Memory ──全局地图──▶ Planner
   ▲                                                                 │
   └────────── /cmd_vel ◀── Control ◀──── /plan ─────────────────────┘
```

闭环：感知 → 世界模型 → 规划 → 控制 → 再回到仿真执行。
