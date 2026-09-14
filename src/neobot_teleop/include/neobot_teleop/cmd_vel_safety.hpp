#pragma once

// 安全门纯函数决策层：无 rclcpp 依赖，输入输出都是普通结构体，便于 gtest。
// 节点（cmd_vel_safety_gate_node.cpp）只负责收集状态填 Request、
// 拿到 Decision 后发布；全部判断都在这里。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace neobot_teleop {

// 停止 / 放行原因，与 /safety_gate/state 发布的字符串一一对应。
enum class StopReason {
  kNone,          // 放行（限速后）
  kEmergencyStop, // 急停闩锁
  kStaleScan,     // 雷达数据超时
  kObstacle,      // 正前方障碍
  kNoCommand,     // 从未收到任何命令
  kStaleCommand,  // 所有命令源都超时
};

// 单个命令源（/cmd_vel_raw 或 /cmd_vel_nav）的最近状态。
struct CommandSourceState {
  geometry_msgs::msg::Twist twist;
  std::int64_t timestamp_ns = 0;  // 最近一条命令的时间戳
  bool received = false;          // 是否收到过该来源的命令
};

struct SafetyGateRequest {
  // 命令源表：topic 名 -> 状态
  std::map<std::string, CommandSourceState> sources;
  std::int64_t now_ns = 0;                 // 当前时钟（纳秒）
  double command_timeout_sec = 0.5;        // 命令看门狗超时
  bool emergency_stop = false;             // 急停闩锁
  bool scan_fresh = false;                 // 雷达数据是否新鲜
  bool obstacle_detected = false;          // 正前方是否检测到障碍
  double linear_speed_limit = 0.0;         // 线速上限，0 = 不限制
  double angular_speed_limit = 0.0;        // 角速上限，0 = 不限制
};

struct SafetyGateDecision {
  geometry_msgs::msg::Twist output;  // 停止时为零速
  StopReason stop_reason = StopReason::kNone;
  std::string active_source;         // kNone 时 = 被放行的命令源 topic
};

inline geometry_msgs::msg::Twist MakeZeroTwist() {
  return geometry_msgs::msg::Twist();
}

// 放行前限速：线速按 hypot 等比缩放（含角速），角速单独钳制。
inline geometry_msgs::msg::Twist ApplySpeedLimit(
    geometry_msgs::msg::Twist twist, double linear_limit,
    double angular_limit) {
  if (linear_limit > 0.0) {
    const double speed = std::hypot(twist.linear.x, twist.linear.y);
    if (speed > linear_limit && speed > 0.0) {
      const double scale = linear_limit / speed;
      twist.linear.x *= scale;
      twist.linear.y *= scale;
      twist.angular.z *= scale;
    }
  }
  if (angular_limit > 0.0) {
    twist.angular.z = std::clamp(twist.angular.z, -angular_limit, angular_limit);
  }
  return twist;
}

// 命令源仲裁：最后写入优先。
// 在"已收到且未超时"的来源中取时间戳最新的一个；
// 返回 topic 名，无可用来源时返回空字符串。
inline std::string SelectActiveSource(const SafetyGateRequest& request) {
  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(request.command_timeout_sec * 1e9);
  std::string best_topic;
  bool found = false;
  for (const auto& [topic, source] : request.sources) {
    if (!source.received) {
      continue;
    }
    if (request.now_ns - source.timestamp_ns > timeout_ns) {
      continue;
    }
    if (!found ||
        source.timestamp_ns > request.sources.at(best_topic).timestamp_ns) {
      best_topic = topic;
      found = true;
    }
  }
  return found ? best_topic : std::string();
}

// 按优先级决策：急停 > 雷达超时 > 前方障碍 > 无命令/命令超时 > 限速放行。
inline SafetyGateDecision EvaluateSafetyGate(const SafetyGateRequest& request) {
  SafetyGateDecision decision;
  if (request.emergency_stop) {
    decision.stop_reason = StopReason::kEmergencyStop;
    return decision;
  }
  if (!request.scan_fresh) {
    decision.stop_reason = StopReason::kStaleScan;
    return decision;
  }
  if (request.obstacle_detected) {
    decision.stop_reason = StopReason::kObstacle;
    return decision;
  }
  const std::string active_topic = SelectActiveSource(request);
  if (active_topic.empty()) {
    bool any_received = false;
    for (const auto& [topic, source] : request.sources) {
      any_received = any_received || source.received;
    }
    decision.stop_reason = any_received ? StopReason::kStaleCommand
                                        : StopReason::kNoCommand;
    return decision;
  }
  decision.stop_reason = StopReason::kNone;
  decision.active_source = active_topic;
  decision.output = ApplySpeedLimit(
      request.sources.at(active_topic).twist, request.linear_speed_limit,
      request.angular_speed_limit);
  return decision;
}

// 与 /safety_gate/state 发布内容一致的字符串描述。
inline std::string DescribeReason(StopReason reason,
                                  const std::string& active_source) {
  switch (reason) {
    case StopReason::kEmergencyStop:
      return "emergency_stop";
    case StopReason::kStaleScan:
      return "stale_scan";
    case StopReason::kObstacle:
      return "obstacle";
    case StopReason::kNoCommand:
      return "no_command";
    case StopReason::kStaleCommand:
      return "stale_command";
    case StopReason::kNone:
      return "forwarding:" + active_source;
  }
  return "unknown";
}

}  // namespace neobot_teleop
