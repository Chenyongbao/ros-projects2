// map_memory_core.cpp
// 地图记忆核心算法实现，通过里程计位姿将随车移动的局部代价地图旋转平移投影融合至全局大地图

#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

/**
 * @brief 构造函数，初始化全局地图大小（150x150）并将所有单元格初始置为 -1（代表未知区域）
 * @param logger ROS 2 日志记录器
 */
MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), global_map_(width_ * height_, -1) {}

/**
 * @brief 缓存最新的局部代价地图
 * @param msg 局部占据代价地图
 */
void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& msg) {
  latest_costmap_ = msg;
  has_costmap_ = true;
}

/**
 * @brief 接收里程计位姿，更新机器人在世界坐标系下的 (x, y) 坐标及航向偏航角 yaw
 * @param msg 里程计消息
 */
void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry& msg) {
  robot_x_ = msg.pose.pose.position.x;
  robot_y_ = msg.pose.pose.position.y;

  // 从四元数提取航向角（Yaw 偏航角）
  const auto& q = msg.pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/**
 * @brief 检查位移条件并尝试执行融合
 * 只有在已获得局部地图，且距离上一次融合位置的平移距离大于 distance_threshold_ 时才真正执行融合计算
 * @return true 完成融合；false 不满足融合条件
 */
bool MapMemoryCore::tryMerge() {
  if (!has_costmap_) return false;

  // 检查自上次融合以来的平移欧式距离
  if (has_last_) {
    double dx = robot_x_ - last_x_;
    double dy = robot_y_ - last_y_;
    double dist = std::sqrt(dx * dx + dy * dy);
    // 未移动足够距离，跳过融合以节省算力
    if (dist < distance_threshold_) return false;
  }

  // 执行坐标变换与栅格融合
  mergeLatestCostmap();

  // 更新上次融合位姿基准点
  last_x_ = robot_x_;
  last_y_ = robot_y_;
  has_last_ = true;
  return true;
}

/**
 * @brief 核心融合函数：遍历局部代价地图各单元格，执行二维刚体变换（旋转+平移）投影到全局大地图
 */
void MapMemoryCore::mergeLatestCostmap() {
  const auto& cm = latest_costmap_;
  const double cm_res = cm.info.resolution;
  const int cm_w = cm.info.width;
  const int cm_h = cm.info.height;
  const double cm_ox = cm.info.origin.position.x;
  const double cm_oy = cm.info.origin.position.y;

  // 在双重循环外预先计算三角函数，大幅优化性能（避免单次融合中重复调用 40000 次 sin/cos）
  const double cos_y = std::cos(robot_yaw_);
  const double sin_y = std::sin(robot_yaw_);

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t value = cm.data[cy * cm_w + cx];
      // 忽略局部地图中未知的栅格 (-1)
      if (value < 0) continue; 

      // 步骤 1：局部栅格索引 -> 局部笛卡尔米制物理坐标（定位在网格中心）
      double local_x = cm_ox + (cx + 0.5) * cm_res;
      double local_y = cm_oy + (cy + 0.5) * cm_res;

      // 步骤 2：局部坐标 -> 世界坐标（按机器人 yaw 航向角旋转，并加上机器人位姿平移）
      double world_x = robot_x_ + local_x * cos_y - local_y * sin_y;
      double world_y = robot_y_ + local_x * sin_y + local_y * cos_y;

      // 步骤 3：世界物理坐标 -> 全局大地图栅格二维索引
      int g_x = static_cast<int>((world_x - origin_x_) / resolution_);
      int g_y = static_cast<int>((world_y - origin_y_) / resolution_);

      // 步骤 4：全局地图边界越界保护
      if (g_x < 0 || g_x >= width_ || g_y < 0 || g_y >= height_) continue;

      // 步骤 5：更新融合策略（保留最大值：障碍物优先级高于自由空间，覆盖未知区域）
      if (value > global_map_[g_y * width_ + g_x]) {
        global_map_[g_y * width_ + g_x] = value;
      }
    }
  }
}

/**
 * @brief 打包生成全局占据栅格地图消息
 */
nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(global_map_.begin(), global_map_.end());
  return msg;
}

}  // namespace robot

