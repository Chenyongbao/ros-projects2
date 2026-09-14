#include "learn_order_interfaces/action/navigate_sequence.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace learn_order_node {

using NavigateSequence = learn_order_interfaces::action::NavigateSequence;
using SequenceGoalHandle = rclcpp_action::ServerGoalHandle<NavigateSequence>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 最小 Nav2 适配器：对外提供学习项目的 NavigateSequence，内部逐个调用 Nav2 NavigateToPose。
// 任务网关只认识 NavigateSequence，因此后续替换导航后端不会改变订单和队列代码。
class Nav2NavigationServer final : public rclcpp::Node {
public:
  Nav2NavigationServer() : Node("nav2_navigation_server") {
    nav2_action_name_ = declare_parameter<std::string>(
        "nav2_action_name", "/navigate_to_pose");
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, nav2_action_name_);

    sequence_server_ = rclcpp_action::create_server<NavigateSequence>(
        this, "/navigate_sequence",
        std::bind(&Nav2NavigationServer::HandleGoal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&Nav2NavigationServer::HandleCancel, this,
                  std::placeholders::_1),
        std::bind(&Nav2NavigationServer::HandleAccepted, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
                "Nav2 adapter ready: /navigate_sequence -> %s",
                nav2_action_name_.c_str());
  }

private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const NavigateSequence::Goal> goal) {
    if (goal->goals.empty()) {
      RCLCPP_WARN(get_logger(), "rejecting empty NavigateSequence goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_) {
      RCLCPP_WARN(get_logger(), "rejecting goal while another sequence is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 在接受回调中立即占用执行权，避免两个并发 Goal 都看到 active=false。
    active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<SequenceGoalHandle>) {
    // Execute 会通过 sequence_handle->is_canceling() 观察这个异步取消请求。
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<SequenceGoalHandle> goal_handle) {
    // 导航过程可能持续很久，不能在 ROS Action 回调线程中同步等待 Nav2。
    std::thread{[this, goal_handle]() { Execute(goal_handle); }}.detach();
  }

  void Execute(const std::shared_ptr<SequenceGoalHandle> sequence_handle) {
    const auto goal = sequence_handle->get_goal();
    const auto total = goal->goals.size();
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(3))) {
      FinishAborted(sequence_handle, "Nav2 NavigateToPose server is unavailable", 0);
      return;
    }

    for (std::size_t index = 0; index < total; ++index) {
      if (sequence_handle->is_canceling()) {
        FinishCanceled(sequence_handle, index);
        return;
      }

      NavigateToPose::Goal nav2_goal;
      nav2_goal.pose = goal->goals[index];
      auto goal_future = nav2_client_->async_send_goal(nav2_goal);
      if (!WaitForGoalHandle(goal_future, sequence_handle, index, total)) {
        return;
      }

      auto nav2_goal_handle = goal_future.get();
      if (!nav2_goal_handle) {
        FinishAborted(sequence_handle, "Nav2 rejected NavigateToPose goal", index);
        return;
      }

      auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
      while (rclcpp::ok()) {
        if (sequence_handle->is_canceling()) {
          // 先取消下游 Nav2，再向上游结束序列 Goal，避免底层仍在运动。
          nav2_client_->async_cancel_goal(nav2_goal_handle);
          FinishCanceled(sequence_handle, index);
          return;
        }

        if (result_future.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::ready) {
          const auto wrapped = result_future.get();
          if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
            FinishAborted(sequence_handle, "Nav2 NavigateToPose failed", index);
            return;
          }
          break;
        }

        PublishFeedback(sequence_handle, index, total, "NAVIGATING");
      }
    }

    FinishSucceeded(sequence_handle, total);
  }

  template <typename FutureT>
  bool WaitForGoalHandle(FutureT& future,
                         const std::shared_ptr<SequenceGoalHandle>& sequence_handle,
                         const std::size_t index, const std::size_t total) {
    while (rclcpp::ok()) {
      if (sequence_handle->is_canceling()) {
        FinishCanceled(sequence_handle, index);
        return false;
      }
      if (future.wait_for(std::chrono::milliseconds(100)) ==
          std::future_status::ready) {
        return true;
      }
      PublishFeedback(sequence_handle, index, total, "WAITING_FOR_NAV2");
    }
    FinishAborted(sequence_handle, "ROS shutdown while waiting for Nav2", index);
    return false;
  }

  void PublishFeedback(const std::shared_ptr<SequenceGoalHandle>& handle,
                       const std::size_t index, const std::size_t total,
                       const std::string& state) {
    auto feedback = std::make_shared<NavigateSequence::Feedback>();
    feedback->current_goal_index = static_cast<std::uint32_t>(index);
    feedback->progress = static_cast<float>(index) / static_cast<float>(total);
    feedback->state = state;
    handle->publish_feedback(feedback);
  }

  void FinishSucceeded(const std::shared_ptr<SequenceGoalHandle>& handle,
                       const std::size_t total) {
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = true;
    result->completed_goals = static_cast<std::uint32_t>(total);
    result->message = "Nav2 sequence completed";
    ClearActive();
    handle->succeed(result);
  }

  void FinishCanceled(const std::shared_ptr<SequenceGoalHandle>& handle,
                      const std::size_t completed) {
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = false;
    result->completed_goals = static_cast<std::uint32_t>(completed);
    result->message = "Nav2 sequence canceled";
    ClearActive();
    handle->canceled(result);
  }

  void FinishAborted(const std::shared_ptr<SequenceGoalHandle>& handle,
                     const std::string& message, const std::size_t completed) {
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = false;
    result->completed_goals = static_cast<std::uint32_t>(completed);
    result->message = message;
    ClearActive();
    handle->abort(result);
  }

  void ClearActive() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_ = false;
  }

  std::string nav2_action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_action::Server<NavigateSequence>::SharedPtr sequence_server_;
  std::mutex state_mutex_;
  bool active_ = false;
};

}  // namespace learn_order_node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<learn_order_node::Nav2NavigationServer>());
  rclcpp::shutdown();
  return 0;
}
