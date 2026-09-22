#ifndef MISSION_MANAGER_NODE_HPP_
#define MISSION_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mission_manager_core.hpp"

/**
 * @class MissionManagerNode
 * @brief 任务管理节点：多点任务队列下发、到达/卡死反馈处理、状态发布
 */
class MissionManagerNode : public rclcpp::Node {
  public:
    MissionManagerNode();

    /// 任务队列回调：收到多点目标，整体替换队列并下发首个目标
    void missionGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /// 到达反馈回调：推进到下一个任务点
    void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /// 卡死告警回调：重试或跳过当前任务点
    void stuckAlertCallback(const std_msgs::msg::Bool::SharedPtr msg);

  private:
    /// 把当前任务点发布到 /goal_point，并刷新 /mission_status
    void dispatchCurrentGoal();

    /// 将核心层状态快照序列化为 JSON 并发布
    void publishStatus();

    robot::MissionManagerCore manager_;  ///< 任务状态机核心

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mission_goals_sub_;   ///< /mission_goals
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;    ///< /goal_reached
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stuck_alert_sub_;     ///< /stuck_alert
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;  ///< /goal_point
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;           ///< /mission_status
};

#endif  // MISSION_MANAGER_NODE_HPP_
