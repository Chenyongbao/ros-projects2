# Costmap 局部代价地图

> 源码：`src/robot/costmap/`（`costmap_node.cpp` + `costmap_core.cpp`）

## 职责

订阅激光雷达的 `/scan`（`sensor_msgs/LaserScan`），把每一帧扫描转换成一张带安全缓冲的局部**占据栅格**（音标 /ˈɒkjəpənsi ˈɡrɛdɪk/：Occupancy Grid 占据栅格：把空间划分为等大小单元格，每个格子存一个代价值表示被占据程度）地图 `/local_costmap`。

核心流程：**清图 → 投影障碍物 → 膨胀 → 发布**。

## 极坐标 → 栅格转换

激光雷达每个光束给出 `(range, angle)` 极坐标，两步转为栅格索引（`convertToGrid`）：

```cpp
// 1. 极坐标 → 笛卡尔坐标
double x_world = range * std::cos(angle);
double y_world = range * std::sin(angle);
// 2. 物理坐标 → 栅格索引（原点偏移 + 分辨率换算）
x_cell = static_cast<int>((x_world - origin_x_) / resolution_);
```

- **resolution**（音标 /ˌrɛzəˈluːʃn/：Resolution 分辨率：一个栅格单元格代表的实际物理尺寸（米/格），本工程为 0.1m）：0.1 m/格
- 越界的点直接丢弃（返回 `false`）

## 单帧处理 `updateFromScan`

1. `initializeCostmap()` —— 每帧清零（局部地图不累积）
2. 遍历 `scan.ranges`，按 `angle_min + i * angle_increment` 还原角度
3. 过滤无效值：`range < range_min || range > range_max || isnan(range)`
4. 命中的格子 `markObstacle()` 置为 `max_cost_`（硬障碍）

## 膨胀算法 `inflateObstacles`

障碍物膨胀（**inflation**（音标 /ɪnˈfleɪʃn/：Inflation 膨胀：在真实障碍物周围按距离衰减叠加代价值，让规划器提前避让的安全机制））分四步：

1. **先收集**所有硬障碍物坐标再涂抹 —— 避免边涂边读造成的级联污染
2. 米制半径换算为格数：`radius_cells = inflation_radius_ / resolution_`
3. 对每个障碍物在正方形邻域内扩散，计算真实**欧氏距离**（音标 /ɪˈklɪdiən ˈdɪstəns/：Euclidean distance 欧氏距离：两点间的直线距离 √(dx²+dy²)）
4. 线性衰减赋值，且**只增不减**（取最大代价）：

```cpp
int new_cost = max_cost_ * (1.0 - dist_meters / inflation_radius_);
if (new_cost > grid_[idx]) grid_[idx] = new_cost;
```

效果：离障碍物越近代价越高，形成一圈"光晕"（halo），让 A* 提前绕行。

## 输出消息 `getOccupancyGrid`

把内部 `std::vector<int8_t>` 打包成 `nav_msgs/OccupancyGrid`，填入 `resolution / width / height / origin`，方向四元数取单位四元数 `(0,0,0,1)` 表示无旋转。

## 参数

见 `config/params.yaml`：地图宽高、分辨率、原点、膨胀半径、最大代价。
