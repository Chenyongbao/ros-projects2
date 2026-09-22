// mission_manager_node.cpp
// 任务管理节点实现：订阅任务队列与执行反馈，下发当前目标并发布任务状态

#include "mission_manager_node.hpp"

MissionManagerNode::MissionManagerNode()
: Node("mission_manager"),
  manager_(this->get_logger(),
           this->declare_parameter<int>("max_retry", 3))
{
  // 订阅多点任务队列（nav_msgs/Path 复用为 waypoint 序列）
  mission_goals_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/mission_goals", 10,
    std::bind(&MissionManagerNode::missionGoalsCallback, this, std::placeholders::_1));

  // 订阅到达反馈（来自 Planner）
  goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/goal_reached", 10,
    std::bind(&MissionManagerNode::goalReachedCallback, this, std::placeholders::_1));

  // 订阅卡死告警（来自 Control）
  stuck_alert_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/stuck_alert", 10,
    std::bind(&MissionManagerNode::stuckAlertCallback, this, std::placeholders::_1));

  // 当前目标发布 → Planner（复用既有话题）
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);

  // 任务状态发布（JSON 文本，Foxglove 文本面板可直接查看）
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_status", 10);

  RCLCPP_INFO(this->get_logger(), "mission_manager 就绪：等待 /mission_goals 任务队列");
}

/**
 * @brief 收到新任务队列：交给核心层，并下发第一个目标
 */
void MissionManagerNode::missionGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::vector<geometry_msgs::msg::PointStamped> goals;
  goals.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    geometry_msgs::msg::PointStamped p;
    p.header = msg->header;
    p.point = pose.pose.position;
    goals.push_back(p);
  }

  manager_.setMissionQueue(goals);
  if (manager_.hasActiveMission()) {
    dispatchCurrentGoal();
  }
  publishStatus();
}

/**
 * @brief 到达反馈：推进队列；还有任务则继续下发
 */
void MissionManagerNode::goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || !manager_.hasActiveMission()) return;

  if (manager_.onGoalReached()) {
    dispatchCurrentGoal();
  }
  publishStatus();
}

/**
 * @brief 卡死告警：重试（重新下发）或跳过（推进到下一个）
 */
void MissionManagerNode::stuckAlertCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || !manager_.hasActiveMission()) return;

  if (manager_.onStuckAlert()) {
    dispatchCurrentGoal();  // 重试：重新下发当前目标
  }
  // 跳过时 onStuckAlert 内部已推进；若还有后续任务同样需要下发
  if (manager_.hasActiveMission() && manager_.getStatus().retry_count == 0) {
    dispatchCurrentGoal();
  }
  publishStatus();
}

/**
 * @brief 下发当前任务点到 /goal_point
 */
void MissionManagerNode::dispatchCurrentGoal() {
  auto goal = manager_.popCurrentGoal();
  goal.header.stamp = this->get_clock()->now();
  goal_pub_->publish(goal);
  RCLCPP_INFO(this->get_logger(), "下发目标点 (%.2f, %.2f)",
              goal.point.x, goal.point.y);
}

/**
 * @brief 状态快照序列化为 JSON 并发布
 */
void MissionManagerNode::publishStatus() {
  const auto s = manager_.getStatus();
  const char* state_str = (s.state == robot::MissionState::ACTIVE) ? "ACTIVE"
                        : (s.state == robot::MissionState::RETRY)  ? "RETRY"
                                                                    : "IDLE";
  std_msgs::msg::String out;
  out.data = "{\"state\":\"" + std::string(state_str) +
             "\",\"current\":" + std::to_string(s.current_index) +
             ",\"total\":" + std::to_string(s.total) +
             ",\"retry\":" + std::to_string(s.retry_count) +
             ",\"finished\":" + std::to_string(s.finished) +
             ",\"skipped\":" + std::to_string(s.skipped) + "}";
  status_pub_->publish(out);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
