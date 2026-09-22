# Map Memory 地图记忆

> 源码：`src/robot/map_memory/`（`map_memory_node.cpp` + `map_memory_core.cpp`）

## 职责

Costmap 输出的局部地图是随车移动的（**ego-centric**（音标 /ˈɛɡəʊ ˈsɛntrɪk/：Ego-centric 以自我为中心：坐标系原点固定在车体上，随车移动）），小车走过一段路后早期感知就被丢掉了。Map Memory 把每一帧局部地图通过**2D 刚体变换**拼接到一张固定大小的全局地图（150×150）上，形成持久化的世界模型 `/global_map`。

## 为什么需要"按位移触发"

如果每帧都融合，大部分计算是重复的（车没动多少，地图没变多少）。`tryMerge()` 设置位移门槛：

```cpp
double dist = std::sqrt(dx*dx + dy*dy);
if (dist < distance_threshold_) return false;  // 移动不够远就跳过
```

只有当自上次融合以来机器人平移距离超过 `distance_threshold_` 才执行融合，节省算力。

## 融合核心 `mergeLatestCostmap`

对局部地图每个非未知（`value >= 0`）栅格执行三步变换：

```text
局部栅格索引 → 局部米制坐标（网格中心 +0.5*res）
            → 世界坐标（按 yaw 旋转 + 机器人位姿平移）   ← 2D 刚体变换
            → 全局栅格索引
```

旋转公式（**yaw**（音标 /jɔː/：Yaw 偏航角：物体绕竖直 Z 轴的旋转角，对地面机器人即"车头朝向"））：

```cpp
world_x = robot_x_ + local_x * cos_y - local_y * sin_y;
world_y = robot_y_ + local_x * sin_y + local_y * cos_y;
```

### 性能细节

`cos(yaw)` / `sin(yaw)` 在双重循环外只算一次。局部地图 200×200 = 40000 格，若放循环内则每次融合多算 4 万次三角函数。

### 融合策略：取最大值

```cpp
if (value > global_map_[g_y * width_ + g_x]) {
    global_map_[g_y * width_ + g_x] = value;
}
```

语义：**障碍物信息优先** —— 一旦某格被观测为障碍，不会被后续的空闲观测覆盖；未知区域（-1）被任何有效观测覆盖。

## 从四元数提取 Yaw

里程计给的是**四元数**（音标 /ˈkwɔːtəniːn/：Quaternion 四元数：用 (x,y,z,w) 四个分量表示三维旋转、避免万向锁的数学工具），提取绕 Z 轴偏航角：

```cpp
robot_yaw_ = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                        1.0 - 2.0 * (q.y*q.y + q.z*q.z));
```

这是标准的四元数 → 欧拉角转换公式（Z-Y-X 顺序中的 Z 分量）。

## 初始化约定

全局地图所有格子初始化为 **-1（未知）**，与 OccupancyGrid 的 ROS 约定一致：-1=未知，0=空闲，100=占据。
