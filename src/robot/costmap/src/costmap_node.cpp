// costmap_node.cpp
// ROS 2 局部代价地图节点实现，订阅激光雷达话题，更新代价地图并向外发布

#include <memory>

#include "costmap_node.hpp"

/**
 * @brief 代价地图节点构造函数
 * 初始化激光雷达订阅者 (/lidar) 与局部代价地图发布者 (/costmap)
 */
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // 订阅激光雷达扫描数据
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // 创建局部占据栅格地图发布者
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

/**
 * @brief 激光雷达扫描消息回调函数
 * 每次收到新一帧雷达扫描时，更新核心代价地图、复用雷达消息的时间戳与坐标系，并将生成的地图发布出去
 * @param scan 激光雷达消息指针
 */
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.updateFromScan(*scan);

  auto grid = costmap_.getOccupancyGrid();
  grid.header = scan->header; // 复用激光扫描的坐标系 frame_id 和时间戳 timestamp
  costmap_pub_->publish(grid);
}

/**
 * @brief 节点主入口函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

