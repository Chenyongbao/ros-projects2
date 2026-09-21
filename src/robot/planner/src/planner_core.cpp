// planner_core.cpp
// 机器人路径规划核心实现（基于占据栅格地图的 8 邻域 A* 搜索算法）

#include <cmath>

#include "planner_core.hpp"

namespace robot
{

/**
 * @brief 构造函数，初始化路径规划核心
 * @param logger ROS 2 日志记录器
 */
PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {} 

/**
 * @brief 更新占据栅格地图数据
 * @param msg 接收到的地图消息 (nav_msgs::msg::OccupancyGrid)
 */
void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid& msg) {
    map_ = msg;
    has_map_ = true;
}

/**
 * @brief 更新目标点位置，并将规划器状态切换为等待到达目标
 * @param msg 目标点位姿消息 (geometry_msgs::msg::PointStamped)
 */
void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped& msg) {
    goal_x_ = msg.point.x;
    goal_y_ = msg.point.y;
    has_goal_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

/**
 * @brief 更新机器人当前在世界坐标系下的位置
 * @param x 当前 X 坐标（米）
 * @param y 当前 Y 坐标（米）
 */
void PlannerCore::updateOdometry(double x, double y) {
    robot_x_ = x;
    robot_y_ = y;
}

/**
 * @brief 将世界坐标系下的连续物理坐标 (wx, wy) 转换为栅格地图的二维网格索引 (cx, cy)
 * @param wx 世界坐标 X（米）
 * @param wy 世界坐标 Y（米）
 * @param cx 转换输出的栅格列索引
 * @param cy 转换输出的栅格行索引
 * @return true 坐标落在地图有效边界内；false 坐标越界
 */
bool PlannerCore::worldToGrid(double wx, double wy, int& cx, int& cy) const {
    // 根据地图原点 (origin) 和分辨率 (resolution) 计算栅格索引
    cx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
    cy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    int w = map_.info.width, h = map_.info.height;
    return cx >= 0 && cx < w && cy >= 0 && cy < h;
}

/**
 * @brief 将栅格地图的网格索引 (cx, cy) 转换回世界坐标系下的物理坐标 (wx, wy)
 * @note 转换后的坐标对准当前栅格的正中心 (+0.5 * resolution)
 */
void PlannerCore::gridToWorld(int cx, int cy, double& wx, double& wy) const {
    wx = map_.info.origin.position.x + (cx + 0.5) * map_.info.resolution;
    wy = map_.info.origin.position.y + (cy + 0.5) * map_.info.resolution;
}

/**
 * @brief 判断指定栅格是否被障碍物占据或不可通行
 * @param cx 栅格列索引
 * @param cy 栅格行索引
 * @return true 该栅格被占据或代价高于阈值；false 可安全通行
 */
bool PlannerCore::isOccupied(int cx, int cy) const {
    int idx = cy * map_.info.width + cx;
    int8_t v = map_.data[idx];
    // 栅格代价值大于等于 25 即视作不可通行的障碍物
    return v >= 25;
}

/**
 * @brief A* 启发式估计函数：计算当前栅格 a 到目标栅格 b 的欧几里得距离
 */
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 执行 A* 算法，在当前栅格地图中寻找一条从机器人当前位置到目标的无碰撞路径
 * @return nav_msgs::msg::Path 包含一系列有序世界坐标 PoseStamped 点的路径
 */
nav_msgs::msg::Path PlannerCore::planPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";

    // 检查地图与目标是否齐备
    if (!has_map_ || !has_goal_) return path;

    // 1. 将起点（机器人当前位置）与终点（目标位置）转换为栅格索引
    int sx, sy, gx, gy;
    if (!worldToGrid(robot_x_, robot_y_, sx, sy)) return path;
    if (!worldToGrid(goal_x_, goal_y_, gx, gy)) return path;
    CellIndex start(sx, sy);
    CellIndex goal(gx, gy);

    // 2. 初始化 A* 算法所需数据结构
    // open: 开放集合（优先队列，小顶堆，按 f_score 排序）
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
    // g_score: 从起点到各个栅格的已知最短路径代价值
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    // came_from: 用于回溯路径的前驱节点映射
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    // 起点代价值初始化
    g_score[start] = 0.0;
    open.emplace(start, heuristic(start, goal));

    // 3. 定义 8 邻域搜索方向偏移及其对应步长移动代价
    // 直行代价为 1.0，斜对角移动代价为 √2 (约 1.414)
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double SQRT2 = std::sqrt(2.0);
    const double step_cost[] = {SQRT2, 1.0, SQRT2, 1.0, 1.0, SQRT2, 1.0, SQRT2};

    const int w = map_.info.width;
    const int h = map_.info.height;

    // 4. A* 主搜索循环
    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        // 成功到达目标点，开始反向回溯并重构路径
        if (current.index == goal) {
            std::vector<CellIndex> cells;
            CellIndex c = goal;
            while (c != start) {
                cells.push_back(c);
                c = came_from[c];
            }
            cells.push_back(start);
            // 将从终点倒序回溯的路径反转为从起点到终点
            std::reverse(cells.begin(), cells.end());

            // 将栅格点转换为世界坐标系下的 PoseStamped 路径点
            for (const auto& cell : cells) {
                double wx, wy;
                gridToWorld(cell.x, cell.y, wx, wy);
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "sim_world";
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            // 路径后处理：快捷化 + 重采样 + 样条平滑（平滑碰撞时自动回退）
            std::vector<PathPoint2D> smoothed = postProcessPath(path.poses);
            if (smoothed.size() != path.poses.size()) {
                path.poses.clear();
                path.poses.reserve(smoothed.size());
                for (const auto& p : smoothed) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "sim_world";
                    pose.pose.position.x = p.x;
                    pose.pose.position.y = p.y;
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                }
            }
            return path;
        }

        // 遍历当前节点的 8 个相邻栅格
        for (int i = 0; i < 8; ++i) {
            int nx = current.index.x + dx[i];
            int ny = current.index.y + dy[i];

            // 越界检查
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            // 障碍物检查
            if (isOccupied(nx, ny)) continue;

            CellIndex neighbour(nx, ny);
            // 计算经过当前节点到达邻居节点的实际累计代价
            double tentative_g = g_score[current.index] + step_cost[i];

            auto it = g_score.find(neighbour);
            // 若发现更优路径或者该邻居尚未被访问
            if (it == g_score.end() || tentative_g < it->second) {
                came_from[neighbour] = current.index;
                g_score[neighbour] = tentative_g;
                double f = tentative_g + heuristic(neighbour, goal);
                open.emplace(neighbour, f);
            }
        }
    }

    // 未找到可行路径，返回空路径
    return path;
}

/**
 * @brief 检查当前是否需要触发路径重新规划
 * 处于前往目标状态且具备有效地图与目标点时返回 true
 */
bool PlannerCore::shouldReplan() const {
    return state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && has_map_ && has_goal_;
}

/**
 * @brief 判断机器人当前位置是否已进入目标点容差半径范围内（0.5米）
 */
bool PlannerCore::goalReached() const {
    double dx = goal_x_ - robot_x_;
    double dy = goal_y_ - robot_y_;
    return std::sqrt(dx * dx + dy * dy) < 0.5;
}

/**
 * @brief 标记已成功抵达目标，切换状态回到等待新目标，并重置目标有效位
 */
void PlannerCore::markGoalReached() {
    state_ = State::WAITING_FOR_GOAL;
    has_goal_ = false;
}

/**
 * @brief 路径后处理三件套：视线快捷化 → 固定弧长重采样 → 自然三次样条平滑
 * 平滑结果逐点做碰撞检测，一旦切进障碍物即回退原始路径（安全优先于平滑）
 */
std::vector<PathPoint2D> PlannerCore::postProcessPath(
  const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) const
{
  // 1. PoseStamped 序列 → 世界坐标点序列
  PointPath2D points;
  points.reserve(raw_path.size());
  for (const auto& pose : raw_path) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  if (points.size() < 3) return points;

  // 2. 视线检测回调：两点连线按地图分辨率步进采样，任一落点栅格被占据则不可通行
  const double resolution = map_.info.resolution;
  auto hasLineOfSight = [this, resolution](const PathPoint2D& a, const PathPoint2D& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    int steps = std::max(1, static_cast<int>(std::ceil(dist / (resolution * 0.5))));
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;
      int cx, cy;
      if (!worldToGrid(a.x + t * dx, a.y + t * dy, cx, cy) || isOccupied(cx, cy)) {
        return false;
      }
    }
    return true;
  };

  // 3. 自由空间检测回调：点所在栅格可通行
  auto isFree = [this](const PathPoint2D& p) {
    int cx, cy;
    return worldToGrid(p.x, p.y, cx, cy) && !isOccupied(cx, cy);
  };

  // 4. 管线：快捷化裁掉锯齿拐角 → 均匀重采样稳定样条参数化 → 样条平滑
  PointPath2D shortcut_path = shortcutter_.shortcut(points, hasLineOfSight);
  PointPath2D resampled = resampler_.resampleAtFixedSpacing(shortcut_path, resolution);
  SmoothingResult2D smoothed = smoother_.smooth(resampled, resolution, resolution, isFree);

  if (smoothed.used_collision_fallback) {
    logger_.warn("路径平滑后发生碰撞，回退为快捷化路径");
    return resampled;
  }
  return smoothed.path;
}

}  // namespace robot