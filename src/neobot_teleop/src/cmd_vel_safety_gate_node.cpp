// 统一安全门节点：/cmd_vel 全系统唯一发布者。
// 决策逻辑在纯函数 EvaluateSafetyGate（cmd_vel_safety.hpp），
// 本节点只负责收集状态、填 Request、发布 Decision。
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "neobot_teleop/cmd_vel_safety.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

class CmdVelSafetyGateNode final : public rclcpp::Node {
public:
  CmdVelSafetyGateNode() : Node("safety_gate") {
    stop_distance_ = declare_parameter<double>("stop_distance", 0.35);
    stop_angle_ = declare_parameter<double>("stop_angle", 0.52);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    // 上限为 0 表示不限制
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.0);

    // 唯一输出：Gazebo 差速插件订阅 /cmd_vel
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    estop_state_pub_ =
        create_publisher<std_msgs::msg::Bool>("/emergency_stop/state", 10);
    gate_state_pub_ =
        create_publisher<std_msgs::msg::String>("/safety_gate/state", 10);

    // 两个命令源：自定义控制器与 Nav2（Jazzy bringup 已把 Nav2 输出 remap 到
    // /cmd_vel_nav），最后写入优先。
    for (const auto& topic : {"/cmd_vel_raw", "/cmd_vel_nav"}) {
      command_subs_[topic] =
          create_subscription<geometry_msgs::msg::Twist>(
              topic, 10, [this, topic](geometry_msgs::msg::Twist::UniquePtr msg) {
                auto& source = sources_[topic];
                source.twist = *msg;
                source.timestamp_ns = now().nanoseconds();
                source.received = true;
              });
    }
    // 激光雷达（传感器话题用 BEST_EFFORT QoS）
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [this](sensor_msgs::msg::LaserScan::UniquePtr msg) { OnScan(*msg); });
    // 急停命令：true 触发、false 解除
    estop_command_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop/command", 10,
        [this](std_msgs::msg::Bool::UniquePtr msg) {
          emergency_stop_ = msg->data;
          if (msg->data) {
            RCLCPP_WARN(get_logger(), "emergency stop enabled via topic");
          } else {
            RCLCPP_INFO(get_logger(), "emergency stop cleared via topic");
          }
        });
    // 运行时线速上限：>0 覆盖参数值，0 还原为参数值
    speed_limit_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/safety/linear_speed_limit", 10,
        [this](std_msgs::msg::Float32::UniquePtr msg) {
          runtime_linear_limit_ = std::max(0.0, static_cast<double>(msg->data));
        });

    enable_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/enable_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          emergency_stop_ = true;
          RCLCPP_WARN(get_logger(), "emergency stop enabled via service");
          response->success = true;
          response->message = "emergency stop active";
        });
    clear_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/clear_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          emergency_stop_ = false;
          RCLCPP_INFO(get_logger(), "emergency stop cleared via service");
          response->success = true;
          response->message = "emergency stop cleared";
        });

    // 50 ms 定时任务：持续发布安全决策（停止时持续发零速，失效安全）
    timer_ =
        create_wall_timer(std::chrono::milliseconds(50),
                          std::bind(&CmdVelSafetyGateNode::Tick, this));

    RCLCPP_INFO(get_logger(),
                "safety gate active: distance=%.2f m, angle=%.2f rad, "
                "limits linear=%.2f m/s, angular=%.2f rad/s",
                stop_distance_, stop_angle_, max_linear_speed_,
                max_angular_speed_);
  }

private:
  // 扫描一帧雷达数据，判断正前方 ±stop_angle 扇形、stop_distance 内是否有障碍。
  void OnScan(const sensor_msgs::msg::LaserScan& scan) {
    last_scan_ns_ = now().nanoseconds();
    scan_received_ = true;
    obstacle_detected_ = false;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range <= 0.0) {
        continue;
      }
      // 射线角度 = 起始角 + 索引 * 角分辨率，再归一化到 (-pi, pi]
      double angle =
          scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      angle = std::atan2(std::sin(angle), std::cos(angle));
      if (std::abs(angle) <= stop_angle_ && range <= stop_distance_) {
        obstacle_detected_ = true;
        break;
      }
    }
  }

  void Tick() {
    const auto now_ns = now().nanoseconds();
    const bool scan_fresh =
        scan_received_ &&
        (now_ns - last_scan_ns_) <=
            static_cast<std::int64_t>(scan_timeout_ * 1e9);

    neobot_teleop::SafetyGateRequest request;
    for (const auto& [topic, source] : sources_) {
      request.sources[topic] = source;
    }
    request.now_ns = now_ns;
    request.command_timeout_sec = command_timeout_;
    request.emergency_stop = emergency_stop_;
    request.scan_fresh = scan_fresh;
    request.obstacle_detected = obstacle_detected_;
    request.linear_speed_limit =
        runtime_linear_limit_ > 0.0 ? runtime_linear_limit_
                                    : max_linear_speed_;
    request.angular_speed_limit = max_angular_speed_;

    const auto decision = neobot_teleop::EvaluateSafetyGate(request);
    cmd_vel_pub_->publish(decision.output);

    std_msgs::msg::Bool estop_state;
    estop_state.data = emergency_stop_;
    estop_state_pub_->publish(estop_state);

    const std::string reason =
        neobot_teleop::DescribeReason(decision.stop_reason,
                                      decision.active_source);
    std_msgs::msg::String gate_state;
    gate_state.data = reason;
    gate_state_pub_->publish(gate_state);

    // 只在决策原因变化时打日志，避免刷屏
    if (reason != last_reason_) {
      last_reason_ = reason;
      if (decision.stop_reason == neobot_teleop::StopReason::kNone) {
        RCLCPP_INFO(get_logger(), "path clear; forwarding %s",
                    decision.active_source.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "publishing zero velocity: %s",
                    reason.c_str());
      }
    }
  }

  double stop_distance_ = 0.35;
  double stop_angle_ = 0.52;
  double scan_timeout_ = 0.5;
  double command_timeout_ = 0.5;
  double max_linear_speed_ = 0.0;
  double max_angular_speed_ = 0.0;

  // 命令源缓存：每个来源各自记录最近一条命令及其时间戳
  std::map<std::string, neobot_teleop::CommandSourceState> sources_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr>
      command_subs_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::int64_t last_scan_ns_ = 0;
  bool scan_received_ = false;
  bool obstacle_detected_ = false;
  bool emergency_stop_ = false;
  double runtime_linear_limit_ = 0.0;
  std::string last_reason_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
