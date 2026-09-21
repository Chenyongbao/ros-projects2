// control_core.cpp
// 机器人底层运动控制核心实现（基于纯追踪/前瞻点 Pure Pursuit 思想的路径跟踪算法）

#include "control_core.hpp"

#include <cmath>

namespace robot
{

/**
 * @brief 构造函数，初始化控制核心
 * @param logger ROS 2 日志记录器，用于控制台输出与调试
 */
ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

/**
 * @brief 更新待跟踪的全局/局部路径
 * @param msg 接收到的路径消息（包含一系列位姿点）
 */
void ControlCore::updatePath(const nav_msgs::msg::Path& msg) {
  path_ = msg;
  has_path_ = true;
}

/**
 * @brief 更新机器人里程计信息，提取当前位置和航向角
 * @param msg 接收到的里程计消息（包含机器人当前位姿与速度）
 */
void ControlCore::updateOdometry(const nav_msgs::msg::Odometry& msg) {
  // 提取机器人在世界/里程计坐标系下的二维坐标 (x, y)
  robot_x_ = msg.pose.pose.position.x;
  robot_y_ = msg.pose.pose.position.y;

  // 从四元数 (x, y, z, w) 中计算机器人绕 Z 轴的航向角（偏航角 Yaw）
  const auto& q = msg.pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/**
 * @brief 计算小车的速度控制指令（线速度与角速度）
 * @return geometry_msgs::msg::Twist 包含线速度与角速度的速度指令
 */
geometry_msgs::msg::Twist ControlCore::computeCommand() {
  geometry_msgs::msg::Twist cmd; // 默认初始化：线速度和角速度均为 0

  // 若尚未收到路径或路径点为空，直接返回静止指令
  if (!has_path_ || path_.poses.empty()) return cmd;

  // 1. 判断是否到达路径终点
  const auto& last = path_.poses.back();
  double dx_end = last.pose.position.x - robot_x_;
  double dy_end = last.pose.position.y - robot_y_;
  double dist_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
  // 当与终点的距离小于设定的容差阈值时，停止运动
  if (dist_to_end < goal_tolerance_) return cmd;

  // 2. 寻找前瞻目标点（Lookahead Point）
  auto target = findLookaheadPoint();
  if (!target.has_value()) return cmd;

  // 3. 计算从机器人当前位置指向前瞻点的向量及其绝对方位角（世界坐标系下）
  double dx = target->pose.position.x - robot_x_;
  double dy = target->pose.position.y - robot_y_;
  double angle_to_target = std::atan2(dy, dx);

  // 4. 计算航向角偏差：目标方位角与当前车头朝向之差（单位：弧度）
  double heading_error = angle_to_target - robot_yaw_;

  // 将航向角偏差归一化到 [-π, π] 区间，确保小车总是沿着最短路径方向旋转
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  // 5. 转向策略与线速度控制
  // 若偏角过大，则停车原地旋转对齐航向，避免转弯半径过大导致冲出路径
  if (std::abs(heading_error) > max_steering_angle_) {
    cmd.linear.x = 0.0; // 线速度归零，执行原地旋转
  } else {
    cmd.linear.x = linear_speed_; // 航向误差较小时，保持设定线速度前进
  }

  // 限制航向角误差在最大允许转向范围内 [-max_steering_angle_, max_steering_angle_]
  heading_error = std::max(-max_steering_angle_, std::min(heading_error, max_steering_angle_));
  
  // 使用比例控制（P 控制，比例增益 Kp = 2.0）计算转向角速度
  cmd.angular.z = 2.0 * heading_error;

  return cmd;
}

/**
 * @brief 在路径中搜索合适的前瞻目标点
 * @return std::optional<geometry_msgs::msg::PoseStamped> 若找到则返回前瞻点，否则返回 std::nullopt
 */
std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() const {
  if (!has_path_ || path_.poses.empty()) return std::nullopt;

  for (const auto& pose : path_.poses) {
    double dx = pose.pose.position.x - robot_x_;
    double dy = pose.pose.position.y - robot_y_;
    double d = std::sqrt(dx * dx + dy * dy);

    // 仅考虑车身前方的路径点（过滤掉车身正负90度以外的后方路径点）
    double angle_to_pose = std::atan2(dy, dx);
    double angle_diff = angle_to_pose - robot_yaw_;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // 偏差角大于 90 度（π/2），说明该点在机器人后方，跳过以防倒车或折返
    if (std::abs(angle_diff) > M_PI_2) continue;

    // 选取第一个距离大于等于前瞻距离的路径点作为目标点
    if (d >= lookahead_dist_) return pose;
  }

  // 如果剩余所有路径点均在前瞻距离内（表明小车已非常接近终点），则以前瞻路径的最后一个点为目标
  return path_.poses.back();
}

/**
 * @brief 检查当前是否存在有效且非空的路径
 * @return true 存在有效路径；false 不存在路径或路径为空
 */
bool ControlCore::hasPath() const {
  return has_path_ && !path_.poses.empty();
}

}  // namespace robot
  
