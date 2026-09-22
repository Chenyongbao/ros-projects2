// control_core.cpp
// 机器人底层运动控制核心实现（基于纯追踪/前瞻点 Pure Pursuit 思想的路径跟踪算法）

#include "control_core.hpp"

#include <cmath>
#include <limits>

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

/**
 * @brief 计算机器人在路径上的投影弧长进度
 * 找到路径上距当前位置最近的投影点，返回该点对应的累计弧长（米）
 * 参考 taorobot 的 progress monitor 实现
 */
double ControlCore::computePathProgress() const {
  if (path_.poses.size() < 2) return 0.0;

  double accumulated = 0.0;
  double best_progress = 0.0;
  double best_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i + 1 < path_.poses.size(); ++i) {
    double x0 = path_.poses[i].pose.position.x;
    double y0 = path_.poses[i].pose.position.y;
    double x1 = path_.poses[i + 1].pose.position.x;
    double y1 = path_.poses[i + 1].pose.position.y;
    double dx = x1 - x0, dy = y1 - y0;
    double seg_len2 = dx * dx + dy * dy;
    double seg_len = std::sqrt(seg_len2);
    if (seg_len < 1e-6) continue;

    // 当前位置在本段上的投影比例（截断到 [0,1]）
    double t = ((robot_x_ - x0) * dx + (robot_y_ - y0) * dy) / seg_len2;
    t = std::max(0.0, std::min(1.0, t));
    double proj_x = x0 + t * dx;
    double proj_y = y0 + t * dy;
    double dist = std::hypot(robot_x_ - proj_x, robot_y_ - proj_y);

    // 记录最近投影点的累计弧长
    if (dist < best_dist) {
      best_dist = dist;
      best_progress = accumulated + t * seg_len;
    }
    accumulated += seg_len;
  }
  return best_progress;
}

/**
 * @brief 更新卡死检测监控器（时间窗 + 位移/目标距离改善双阈值）
 * 判定条件：窗口期内一直在下发运动指令，但位移和到目标距离的改善都低于阈值
 * @return true 本周期判定为卡死
 */
bool ControlCore::updateStuckDetection(double now, const geometry_msgs::msg::Twist& cmd) {
  // 计算到当前路径终点的距离（作为目标距离改善的度量）
  double goal_dist = std::numeric_limits<double>::infinity();
  if (has_path_ && !path_.poses.empty()) {
    const auto& last = path_.poses.back().pose.position;
    goal_dist = std::hypot(last.x - robot_x_, last.y - robot_y_);
  }

  // 首次调用或刚重置：记录窗口起点
  if (!monitor_.initialized) {
    monitor_.initialized = true;
    monitor_.window_start_x = robot_x_;
    monitor_.window_start_y = robot_y_;
    monitor_.window_start_time = now;
    monitor_.window_start_goal_dist = goal_dist;
    monitor_.is_stuck = false;
    return false;
  }

  // 正在正常下发运动指令才检测（排除到达停车、等待路径等情况）
  bool commanding = std::abs(cmd.linear.x) > 1e-3 || std::abs(cmd.angular.z) > 1e-3;
  if (!commanding) {
    monitor_.window_start_x = robot_x_;
    monitor_.window_start_y = robot_y_;
    monitor_.window_start_time = now;
    monitor_.window_start_goal_dist = goal_dist;
    monitor_.is_stuck = false;
    return false;
  }

  // 窗口未到期，沿用上一次判定
  if (now - monitor_.window_start_time < stuck_window_) {
    return monitor_.is_stuck;
  }

  // 窗口到期：双阈值判定
  double moved = std::hypot(robot_x_ - monitor_.window_start_x,
                            robot_y_ - monitor_.window_start_y);
  double improvement = monitor_.window_start_goal_dist - goal_dist;
  monitor_.is_stuck = moved < min_progress_dist_ && improvement < min_goal_improvement_;

  // 滑动窗口前移
  monitor_.window_start_x = robot_x_;
  monitor_.window_start_y = robot_y_;
  monitor_.window_start_time = now;
  monitor_.window_start_goal_dist = goal_dist;

  if (monitor_.is_stuck) {
    RCLCPP_WARN(logger_, "卡死检测触发：窗口内位移 %.3fm，目标距离改善 %.3fm", moved, improvement);
  }
  return monitor_.is_stuck;
}

/**
 * @brief 计算脱困摆动指令：反转线速度与角速度，尝试倒退脱困
 * 摆动阶段结束后恢复正常的 computeCommand 输出
 */
geometry_msgs::msg::Twist ControlCore::computeRecoveryCommand(double now) {
  geometry_msgs::msg::Twist cmd;
  if (now >= recovery_until_) {
    recovery_until_ = 0.0;  // 摆动结束，恢复正常控制
    return cmd;
  }
  // 反转巡航线速度 + 反向慢速旋转（倒退并偏转，尝试脱离障碍物卡滞）
  cmd.linear.x = -0.5 * linear_speed_;
  cmd.angular.z = 0.5 * max_steering_angle_;
  return cmd;
}

}  // namespace robot
  
