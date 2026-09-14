// 安全门纯函数决策层单测：与 12 项运行验收场景在决策层一一对应。
#include <string>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist.hpp"
#include "neobot_teleop/cmd_vel_safety.hpp"

namespace {

using neobot_teleop::CommandSourceState;
using neobot_teleop::DescribeReason;
using neobot_teleop::SafetyGateRequest;
using neobot_teleop::StopReason;

constexpr std::int64_t kNowNs = 1'000'000'000LL;

geometry_msgs::msg::Twist MakeTwist(double linear_x, double angular_z = 0.0) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.angular.z = angular_z;
  return twist;
}

// age_ns：命令距今多久前到达（纳秒）
CommandSourceState FreshSource(double linear_x, std::int64_t age_ns) {
  CommandSourceState source;
  source.twist = MakeTwist(linear_x);
  source.timestamp_ns = kNowNs - age_ns;
  source.received = true;
  return source;
}

// 默认：雷达新鲜，/cmd_vel_raw 有一条 0.1 s 前的 0.5 m/s 命令
SafetyGateRequest BaseRequest() {
  SafetyGateRequest request;
  request.now_ns = kNowNs;
  request.command_timeout_sec = 0.5;
  request.scan_fresh = true;
  request.sources["/cmd_vel_raw"] = FreshSource(0.5, 100'000'000);
  return request;
}

}  // namespace

// 场景 9：急停优先级最高，压过雷达超时与障碍
TEST(SafetyGateTest, EmergencyStopHasTopPriority) {
  auto request = BaseRequest();
  request.emergency_stop = true;
  request.scan_fresh = false;
  request.obstacle_detected = true;
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kEmergencyStop);
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(decision.output.angular.z, 0.0);
}

// 场景 6：雷达超时优先于障碍
TEST(SafetyGateTest, StaleScanStopsBeforeObstacle) {
  auto request = BaseRequest();
  request.scan_fresh = false;
  request.obstacle_detected = true;
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kStaleScan);
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.0);
}

// 场景 5：正前方障碍 -> 零速
TEST(SafetyGateTest, ObstacleStops) {
  auto request = BaseRequest();
  request.obstacle_detected = true;
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kObstacle);
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.0);
}

// 场景 2：雷达正常但从未收到任何命令
TEST(SafetyGateTest, NoCommandWhenNeverReceived) {
  auto request = BaseRequest();
  request.sources.clear();
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kNoCommand);
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.0);
}

// 场景 8：收到过命令但全部超时
TEST(SafetyGateTest, StaleCommandWhenExpired) {
  auto request = BaseRequest();
  request.sources["/cmd_vel_raw"] = FreshSource(0.5, 600'000'000);  // 0.6 s > 0.5 s
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kStaleCommand);
}

// 边界：恰好等于超时不算 stale
TEST(SafetyGateTest, CommandAtTimeoutBoundaryStillActive) {
  auto request = BaseRequest();
  request.sources["/cmd_vel_raw"] = FreshSource(0.5, 500'000'000);
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kNone);
}

// 场景 3：单命令源直通
TEST(SafetyGateTest, ForwardingPassesCommandThrough) {
  const auto request = BaseRequest();
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kNone);
  EXPECT_EQ(decision.active_source, "/cmd_vel_raw");
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.5);
}

// 场景 4：两个命令源共存，取更新的一条（最后写入优先）
TEST(SafetyGateTest, ArbitrationPicksNewestSource) {
  auto request = BaseRequest();
  request.sources["/cmd_vel_nav"] = FreshSource(0.3, 50'000'000);  // 比 raw 新
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_EQ(decision.stop_reason, StopReason::kNone);
  EXPECT_EQ(decision.active_source, "/cmd_vel_nav");
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 0.3);
}

// 场景 11：线速按 hypot 等比缩放（|v| = 0.5 缩到 0.25）
TEST(SafetyGateTest, LinearLimitScalesByHypot) {
  auto request = BaseRequest();
  request.linear_speed_limit = 0.25;
  request.sources["/cmd_vel_raw"] = FreshSource(0.3, 100'000'000);
  request.sources["/cmd_vel_raw"].twist.linear.y = 0.4;
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_NEAR(decision.output.linear.x, 0.15, 1e-9);
  EXPECT_NEAR(decision.output.linear.y, 0.20, 1e-9);
}

// 角速单独钳制（负方向）
TEST(SafetyGateTest, AngularLimitClamps) {
  auto request = BaseRequest();
  request.angular_speed_limit = 0.6;
  request.sources["/cmd_vel_raw"].twist.angular.z = -1.0;
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_NEAR(decision.output.angular.z, -0.6, 1e-9);
}

// 上限为 0 表示不限制
TEST(SafetyGateTest, ZeroLimitMeansUnlimited) {
  auto request = BaseRequest();
  request.sources["/cmd_vel_raw"] = FreshSource(2.0, 100'000'000);
  const auto decision = neobot_teleop::EvaluateSafetyGate(request);
  EXPECT_DOUBLE_EQ(decision.output.linear.x, 2.0);
}

// /safety_gate/state 的字符串与验收脚本断言保持一致
TEST(SafetyGateTest, DescribeReasonMatchesGateStateStrings) {
  EXPECT_EQ(DescribeReason(StopReason::kEmergencyStop, ""), "emergency_stop");
  EXPECT_EQ(DescribeReason(StopReason::kStaleScan, ""), "stale_scan");
  EXPECT_EQ(DescribeReason(StopReason::kObstacle, ""), "obstacle");
  EXPECT_EQ(DescribeReason(StopReason::kNoCommand, ""), "no_command");
  EXPECT_EQ(DescribeReason(StopReason::kStaleCommand, ""), "stale_command");
  EXPECT_EQ(DescribeReason(StopReason::kNone, "/cmd_vel_nav"),
            "forwarding:/cmd_vel_nav");
}
