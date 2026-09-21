// planner_node.cpp
// ROS 2 路径规划节点实现，负责接收地图、目标点与定位信息，并在地图更新时触发 A* 路径规划

#include <chrono>

#include "planner_node.hpp"

/**
 * @brief 规划节点构造函数
 * 初始化订阅者（地图 /map、目标点 /goal_point、里程计 /odom/filtered）、路径发布者（/path）以及状态检测定时器
 */
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // 订阅全局占据栅格地图
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  // 订阅导航目标点（通常由 RViz 的 "Clicked Goal" 工具发布）
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  // 订阅机器人滤波后的里程计位姿
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  // 创建路径发布者，将规划生成的路径发送给控制器模块
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // 创建定时器：每隔 500ms（频率 2Hz）检查是否已到达目标点
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );
}

/**
 * @brief 接收到新地图时的回调函数
 * @param msg 地图消息指针
 * @note 当小车正处于前往目标的状态时，每次地图更新都会立即触发局部/全局重规划
 */
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.updateMap(*msg);

  // 如果小车正处于前往目标的状态，则在最新地图上立即执行重规划以避开新障碍物
  if (planner_.shouldReplan()) {
    auto path = planner_.planPath();
    path.header.stamp = this->get_clock()->now();
    path_pub_->publish(path);
  }
}

/**
 * @brief 接收到新目标点时的回调函数
 * @param msg 目标点位姿指针
 */
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_.updateGoal(*msg);
}

/**
 * @brief 接收到里程计位姿时的回调函数
 * @param msg 里程计消息指针
 */
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.updateOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y);
} 

/**
 * @brief 周期性定时器回调函数
 * 检查小车是否已经抵达目标点。到达后重置规划器状态并打印日志提示
 */
void PlannerNode::timerCallback() {
  if (!planner_.shouldReplan()) return;

  // 检查小车与目标的距离是否小于设定阈值
  if (planner_.goalReached()) {
    RCLCPP_INFO(this->get_logger(), "已成功到达导航目标点！(Goal reached!)");
    planner_.markGoalReached(); // 将规划器状态切回 WAITING_FOR_GOAL
    return;
  }

  // 路径重新规划已由地图回调事件驱动触发，无需在此重复计算
}

/**
 * @brief 节点入口主函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

