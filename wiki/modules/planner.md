# Planner A\* 路径规划

> 源码：`src/robot/planner/`（`planner_node.cpp` + `planner_core.cpp`）

## 职责

接收全局地图 `/global_map`、里程计位姿和用户点击的目标点 `/clicked_point`，在占据栅格上运行 **8 邻域 A\*** 搜索，输出从机器人当前位置到目标的无碰撞路径 `/plan`。

## 状态机

```text
WAITING_FOR_GOAL ── 收到目标点 ──▶ WAITING_FOR_ROBOT_TO_REACH_GOAL
        ▲                                    │
        └────────── goalReached() ───────────┘
```

- `shouldReplan()`：处于"前往目标"状态且地图、目标齐备时触发规划
- `goalReached()`：与目标距离 < 0.5 m 判定到达

## 坐标互转

```cpp
// 世界坐标 → 栅格索引
cx = (wx - map.info.origin.position.x) / map.info.resolution;
// 栅格索引 → 世界坐标（对准网格中心）
wx = origin.x + (cx + 0.5) * resolution;
```

## A\* 算法实现

A\*（**A-star**（音标 /eɪ stɑːr/：A\* 搜索：在图中用 f(n)=g(n)+h(n) 评估代价、保证启发式可采纳时找到最短路径的图搜索算法））的三要素在本工程中的落地：

| 要素 | 数据结构 | 说明 |
|---|---|---|
| **Open Set**（开放集合：待探索节点的优先队列（小顶堆），按 f 值排序） | `std::priority_queue` | 按 `f_score` 排序的小顶堆 |
| **g-score**（g(n)：从起点到节点 n 的实际累计代价） | `unordered_map<CellIndex,double>` | 起点到各格的已知最短代价 |
| **Heuristic**（音标 /hjʊəˈrɪstɪk/：启发函数 h(n)：对"到目标还差多远"的乐观估计，本工程用欧氏距离） | `heuristic()` | 欧氏距离，可采纳（admissible）保证最优 |

主循环（`planPath()`）：

1. 起点入堆，`g_score[start] = 0`
2. 弹出 f 最小的节点；若是目标则**回溯** `came_from` 链重构路径
3. 否则展开 8 邻域：
   - 直行步长代价 1.0，斜对角 √2（≈1.414）
   - 越界或 `isOccupied`（代价 ≥ 25）的邻居跳过
   - `tentative_g = g_score[current] + step_cost`，若比已记录的更优则更新 `g_score` / `came_from` 并入堆
4. 堆空仍未达目标 → 返回空路径（不可达）

### 为什么 8 邻域 + √2

4 邻域路径呈锯齿状楼梯形；8 邻域允许斜走，路径更短更平滑。对角步长必须乘 √2 才能保持 g 值与真实欧氏长度一致，否则会系统性低估对角路径代价。

## 障碍判定阈值

```cpp
int8_t v = map_.data[idx];
return v >= 25;  // ≥25 视为不可通行
```

阈值 25 而非 100：Map Memory 融合后的膨胀光晕值也被计入，低阈值让小车与膨胀区保持距离，配合 Costmap 的线性衰减实现渐进避让。

## 输出

栅格路径经 `gridToWorld` 转回 `sim_world` 坐标系的 `nav_msgs/Path`（`PoseStamped` 序列）。输出前会经过**路径后处理管线**（视线快捷化 → 固定弧长重采样 → 三次样条平滑，带碰撞回退），详见 [路径后处理：平滑三件套](./path-smoother.md)。
