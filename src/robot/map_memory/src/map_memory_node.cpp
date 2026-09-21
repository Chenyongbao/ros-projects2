// map_memory_node.cpp
// ROS 2 地图记忆节点实现，订阅代价地图与滤波里程计，定期将局部地图融合入全局地图并向 /map 发布

#include <chrono>

#include "map_memory_node.hpp"

/**
 * @brief 地图记忆节点构造函数
 * 初始化话题订阅者（/costmap, /odom/filtered）、全局地图发布者（/map）与 1Hz 融合定时器
 */
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // 订阅局部代价地图
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  // 订阅滤波里程计以获得小车在世界坐标系下的基准位姿
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // 创建全局占据栅格地图发布者
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // 创建定时器：每隔 1 秒触发一次融合与发布
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this)
  );
}

/**
 * @brief 接收到局部代价地图时的回调函数
 */
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_memory_.updateCostmap(*msg);
}

/**
 * @brief 接收到里程计位姿时的回调函数
 */
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  map_memory_.updateOdometry(*msg);
}

/**
 * @brief 定时器回调函数：尝试融合局部地图并发布最新的全局地图
 */
void MapMemoryNode::updateMap() {
  // 尝试融合（内部会判断小车自上次融合以来的位移是否大于距离阈值）
  map_memory_.tryMerge();

  // 获取全局地图消息，更新时间戳和参考坐标系（sim_world 全局世界系），并发布
  auto grid = map_memory_.getGlobalMap();
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "sim_world"; // 全局世界坐标系
  map_pub_->publish(grid);
}

/**
 * @brief 节点主入口函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

