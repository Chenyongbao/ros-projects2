// costmap_core.cpp
// 局部代价地图核心算法实现，负责处理激光雷达扫描并生成带有安全膨胀半径的占据栅格图

#include "costmap_core.hpp"

#include <cmath>

namespace robot
{

/**
 * @brief 构造函数，初始化日志记录器并分配栅格地图缓冲区内存（全部初始化为 0）
 * @param logger ROS 2 日志记录器
 */
CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), grid_(width_ * height_, 0) {}

/**
 * @brief 初始化/重置栅格地图，将所有网格代价值清零（表示无障碍物的自由空间）
 */
void CostmapCore::initializeCostmap() {
  grid_.assign(width_ * height_, 0);
}

/**
 * @brief 将极坐标系的激光测距点转换为机器人局部坐标系下的栅格索引
 * @param range 激光测距距离（米）
 * @param angle 激光测量角度（弧度）
 * @param x_cell 输出网格横坐标（列索引）
 * @param y_cell 输出网格纵坐标（行索引）
 * @return true 栅格索引在地图边界内；false 越界
 */
bool CostmapCore::convertToGrid(double range, double angle, int& x_cell, int& y_cell) const {
  // 1. 极坐标转直角笛卡尔物理坐标 (x_world, y_world)
  double x_world = range * std::cos(angle);
  double y_world = range * std::sin(angle);

  // 2. 根据地图原点偏移和分辨率计算对应的网格单元格坐标
  x_cell = static_cast<int>((x_world - origin_x_) / resolution_);
  y_cell = static_cast<int>((y_world - origin_y_) / resolution_);

  // 3. 检查坐标是否落在 [0, width_) 和 [0, height_) 范围内
  return (x_cell >= 0 && x_cell < width_ && y_cell >= 0 && y_cell < height_);
}

/**
 * @brief 将指定栅格标记为最高代价的硬障碍物
 * @param x_cell 栅格列索引
 * @param y_cell 栅格行索引
 */
void CostmapCore::markObstacle(int x_cell, int y_cell) {
  grid_[y_cell * width_ + x_cell] = max_cost_;
}

/**
 * @brief 处理激光雷达单帧扫描数据：清空旧地图、投影障碍物并执行膨胀
 * @param scan ROS 2 激光雷达扫描消息
 */
void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan) {
  // 每帧开始前先重置清空局部地图
  initializeCostmap();

  // 遍历激光雷达返回的所有激光测距光束
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;

    // 过滤超出量程范围或无效（NaN/Inf）的雷达测量值
    if (range < scan.range_min || range > scan.range_max || std::isnan(range)) {
      continue;
    }

    int x_cell, y_cell;
    // 若点落在地图范围内，将其标记为硬障碍物
    if (convertToGrid(range, angle, x_cell, y_cell)) {
      markObstacle(x_cell, y_cell);
    }
  }

  // 执行障碍物膨胀，为小车提供避障安全缓冲区
  inflateObstacles();
}

/**
 * @brief 障碍物膨胀算法
 * 遍历所有硬障碍物栅格，在膨胀半径内按线性距离衰减模型为周围自由栅格赋予递减的膨胀代价值
 */
void CostmapCore::inflateObstacles() {
  // 1. 预先收集所有硬障碍物的网格坐标，避免在膨胀过程中产生级联污染
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y * width_ + x] == max_cost_) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  // 2. 将米制膨胀半径换算为网格单元格数（例如 1.0m / 0.1m = 10 个单元格）
  int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

  // 3. 对每个障碍物单元格，向外扩散涂抹安全光晕（Halo）
  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;

        // 越界检查
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        // 计算当前邻居栅格到障碍物中心的真实欧式物理距离（米）
        double dist_cells = std::sqrt(dx * dx + dy * dy);
        double dist_meters = dist_cells * resolution_;
        if (dist_meters > inflation_radius_) continue;

        // 依据距离线性递减计算膨胀代价：离障碍物越近代价越接近 max_cost_
        int new_cost = static_cast<int>(max_cost_ * (1.0 - dist_meters / inflation_radius_));

        // 只有当新计算的膨胀代价高于单元格已有代价时才更新（取最大代价）
        int idx = ny * width_ + nx;
        if (new_cost > grid_[idx]) {
          grid_[idx] = new_cost;
        }
      }
    }
  }
}

/**
 * @brief 构建并返回 ROS 2 标准 OccupancyGrid 地图消息
 */
nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0; // 单位四元数（无旋转）
  msg.data.assign(grid_.begin(), grid_.end()); // 将一维容器数据复制到消息中
  return msg;
}

}  // namespace robot