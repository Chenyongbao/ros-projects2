# 技术方案：任务状态机 + 恢复行为（第一阶段改造）

> 目标：把"点到点移动器"升级为"能接多点配送任务的机器人业务层"。
> 新增 1 个节点 + 增强 1 个节点，不改现有四节点间的话题协议。

## 一、总体架构

```text
                       ┌──────────────────────────┐
   /mission_goals      │  mission_manager (新增)   │
  (多点任务队列) ──────▶│  任务队列 · 状态机 · 重试  │
                       └────┬───────────────┬─────┘
                            │ 当前目标        │ 任务状态
                            ▼                ▼
                     /goal_point        /mission_status
                            │                ▲
                            ▼                │
   ┌──────────┐  /path  ┌──────────┐  /odom  │
   │ Planner  │───────▶ │ Control  │─────────┤
   └──────────┘         │ +卡死检测 │         │
        ▲               └────┬─────┘         │
        │ /replan_req        │ /cmd_vel      │
        └────────────────────┤               │
                             ▼               │
   ┌──────────┐ /global_map  (仿真底盘)       │
   │MapMemory │  ...（其余链路不变）          │
   └──────────┘◀── /stuck_alert ──────────────┘
```

## 二、新节点 mission_manager

**职责**：持有任务队列，逐个下发目标点，监听执行结果，失败/卡死时按策略重试或跳过，对外发布任务状态。

### 状态机

```text
IDLE ──入队──▶ ACTIVE ──到达──▶ (队列非空) ACTIVE(下一个) / (空) IDLE
                │  ▲
      重试次数<max │  │ 重新下发
                ▼  │
             RETRY ──次数耗尽──▶ 跳过该点 → (队列非空) ACTIVE / (空) IDLE(报告失败)
```

- `IDLE`：无任务。收到 `/mission_goals`（`nav_msgs/Path` 复用为 waypoint 序列）入队 → 取首个下发
- `ACTIVE`：已下发当前目标，等待 Control 的到达反馈
- `RETRY`：收到 `/stuck_alert` 或重规划仍失败 → 计数 +1 重新下发同一目标；超过 `max_retry` → 跳过，继续下一任务点

### 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/mission_goals` | `nav_msgs/Path` | 订阅 | 多点任务队列（用户/Foxglove 一次发多点） |
| `/goal_point` | `geometry_msgs/PointStamped` | 发布 | 当前任务目标 → Planner（复用现有话题） |
| `/mission_status` | `std_msgs/String` | 发布 | JSON 状态：`{"state":"ACTIVE","current":2,"total":5,"retry":1}` |
| `/goal_reached` | `std_msgs/Bool` | 订阅 | 到达反馈（Planner 已有 goalReached 逻辑，改为发布确认） |
| `/stuck_alert` | `std_msgs/Bool` | 订阅 | 卡死告警（来自 Control） |

> 决策：不复用 `/clicked_point`，避免与手动单点导航互相打断；Planner 的 goal 话题名保持不变，mission_manager 只是它的另一个"客户"。

## 三、Control 节点增强：卡死检测 + 脱困

移植 taorobot 的进度监控思路，落在 `control_core`（纯算法，可单测）：

### 卡死判定（时间窗 + 双阈值）

1. 每个控制周期计算**路径投影弧长进度**（当前位姿到路径的最近投影点累计弧长）
2. 滑动窗口（默认 3s）到期时检查：
   - 窗口内**位移** < `min_progress_distance`（0.05m）**且**
   - 窗口内**到目标距离改善** < `min_goal_improvement`（0.05m）
   - 且期间**一直在下发非零指令**（排除正常停车）
3. 满足 → 置 `is_stuck`，发布 `/stuck_alert=true`

### 脱困行为（两级）

- **第一级 · 原地摆动**：卡死后 1.5s 内反转指令（线速度取反 + 角速度反向），尝试脱困；期间发布 `/replan_req`
- **第二级 · 触发重规划**：Planner 收到 `/replan_req` 强制 `shouldReplan()`，在最新全局地图上重规划绕行
- 摆动后窗口进度仍为零 → 上报 `/stuck_alert` 给 mission_manager 走 RETRY 流程

## 四、文件改动清单

| 文件 | 动作 |
|---|---|
| `src/robot/mission_manager/`（新包） | node + core 双层：`mission_manager_core.hpp/cpp`（状态机纯逻辑）+ node |
| `src/robot/control/include/control_core.hpp` | 增加 ProgressMonitor 结构体（窗口起点位姿/时间、is_stuck）与 `updateProgressMonitor()` |
| `src/robot/control/src/control_core.cpp` | 实现投影弧长进度 + 双阈值判定 |
| `src/robot/control/src/control_node.cpp` | 定时器里调用监控，卡死时发布 `/stuck_alert`、执行摆动指令 |
| `src/robot/planner/src/planner_node.cpp` | 订阅 `/replan_req` 强制重规划；到达后发布 `/goal_reached` |
| `src/robot/bringup_robot/launch/robot.launch.py` | 挂载 mission_manager 节点 |

## 五、验证方式

- 编译：WSL `colcon build`（Docker 装好后 `./watod build`）
- 功能：Foxglove 连发 3 个任务点 → 观察依次到达 + `/mission_status` 状态流转
- 恢复：在路径中间放障碍 → 观察卡死告警 → 摆动/重规划绕行或跳过该点继续任务

## 六、简历表述（做完即可写）

> 设计多点配送任务管理层：任务队列状态机（IDLE/ACTIVE/RETRY）+ 双阈值卡死检测（路径投影弧长进度 + 时间窗位移）+ 两级脱困（原地摆动、触发重规划），任务失败自动跳转，实现无人干预的多点配送闭环
