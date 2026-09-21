// control_node.cpp
// ROS 2 控制节点实现，负责接收路径规划结果与里程计，并周期性计算下发小车速度控制指令

#include <chrono>

#include "control_node.hpp"

/**
 * @brief 控制节点构造函数
 * 初始化订阅者（路径 /path、滤波后里程计 /odom/filtered）、速度发布者（/cmd_vel）以及控制定时器
 */
ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // 订阅规划器发布的全局/局部路径
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  // 订阅定位模块/EKF滤波后的里程计数据
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  // 创建速度指令发布者，将计算出的控制速度发布至底盘驱动话题 /cmd_vel
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 创建控制循环定时器：每隔 100ms（频率 10Hz）执行一次控制逻辑
  timer_  = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this)
  );
}

/**
 * @brief 接收到新路径时的回调函数
 * @param msg 路径消息智能指针
 */
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.updatePath(*msg);
}

/**
 * @brief 接收到里程计更新时的回调函数
 * @param msg 里程计消息智能指针
 */
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  control_.updateOdometry(*msg);
}

/**
 * @brief 定时控制主循环
 * 检查是否有路径，若存在则计算跟踪速度指令并向 /cmd_vel 发布；若无路径则不发布以允许遥控(teleop)接管
 */
void ControlNode::controlLoop() {
  // 若无有效路径，直接退出，避免发布零速覆盖遥控操作
  if (!control_.hasPath()) return;

  // 调用控制算法核心计算速度控制指令
  auto cmd = control_.computeCommand();
  // 发布线速度与角速度指令
  twist_pub_->publish(cmd);
}

/**
 * @brief 节点入口函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

