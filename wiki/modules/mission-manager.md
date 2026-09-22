# Mission Manager 任务管理层

> 源码：`src/robot/mission_manager/`（新增包）+ `control` 的卡死检测增强

## 职责

把"点到点移动器"升级为**能接多点配送任务的机器人业务层**：任务队列、失败重试、卡死脱困、状态上报。这是"能演示"和"能交付"的分界线——真实产品必须能应对动态障碍导致的卡滞，并在任务失败时不中断整批任务。

## 任务状态机

```text
IDLE ──收到 /mission_goals──▶ ACTIVE ──到达──▶ 队列非空 ? ACTIVE(下一个) : IDLE
                                │
                      收到 /stuck_alert
                                ▼
                              RETRY ──重试次数 < max──▶ 重新下发当前目标
                                │
                          次数耗尽 ──▶ 跳过该点 → ACTIVE(下一个) / IDLE
```

核心类 `MissionManagerCore`（`mission_manager_core.hpp/cpp`）是**纯逻辑状态机**，不含任何 ROS 通信——订阅发布全部在 node 层，沿用项目 `node + core` 分层惯例，可独立单测。

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/mission_goals` | `nav_msgs/Path` | 订阅 | 多点任务队列（复用 Path 为 waypoint 序列） |
| `/goal_point` | `geometry_msgs/PointStamped` | 发布 | 当前任务目标 → Planner（复用既有话题） |
| `/goal_reached` | `std_msgs/Bool` | 订阅 | 到达反馈（Planner 在 goalReached 时发布） |
| `/stuck_alert` | `std_msgs/Bool` | 订阅 | 卡死告警（来自 Control） |
| `/mission_status` | `std_msgs/String` | 发布 | JSON 状态：`{"state":"ACTIVE","current":2,"total":5,"retry":1,"finished":1,"skipped":0}` |

## 卡死检测与脱困（Control 增强）

移植自 taorobot 的 progress monitor 思路，落在 `control_core`（纯算法层）：

### 双阈值判定

每个控制周期（10Hz）检查，**时间窗 3s** 到期时判定：

1. 窗口内**位移** < 0.05m（位置没怎么动）
2. 窗口内**到目标距离改善** < 0.05m（也没在接近目标）
3. 且期间一直**在下发非零运动指令**（排除正常停车等待）

三者同时满足 → 判定卡死。

### 脱困两级响应

1. **原地倒退摆动**（1.5s）：反转线速度 + 反向慢速旋转，尝试脱离卡滞
2. **上报 `/stuck_alert`**：mission_manager 收到后走 RETRY 流程重新下发目标，Planner 在最新地图上重规划绕行

摆动结束后重置监控窗口，避免连续误报。

## 路径投影弧长进度

卡死检测的核心度量：把机器人当前位置**投影**到路径最近线段上，取该投影点的累计弧长作为"跟踪进度"（`computePathProgress()`）。相比只看位移，它能区分"沿路径前进"和"原地打转"。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_retry` | 3 | 单个任务点最大重试次数，超过跳过 |
| 卡死时间窗 | 3s | 双阈值判定的观测窗口 |
| 位移阈值 | 0.05m | 窗口内最小位移 |
| 目标改善阈值 | 0.05m | 窗口内到目标距离最小改善量 |
| 摆动时长 | 1.5s | 脱困倒退持续时间 |

## 简历表述

> 设计多点配送任务管理层：任务队列状态机（IDLE/ACTIVE/RETRY）+ 双阈值卡死检测（时间窗位移 + 目标距离改善）+ 两级脱困（原地摆动、重试重规划），任务失败自动跳转，实现无人干预的多点配送闭环。
