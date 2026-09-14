#include "learn_order_interfaces/action/navigate_sequence.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace learn_order_node {

using NavigateSequence = learn_order_interfaces::action::NavigateSequence;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateSequence>;

// 这是一个只模拟时序的导航 Action Server。
// 它不读取地图、不规划路径、不发布 cmd_vel，只用于验证调度器和 Action 的连接。
class MockNavigationServer final : public rclcpp::Node {
public:
  MockNavigationServer() : Node("mock_navigation_server") {
    // 设为 true 时，第一次 Goal 会故意失败，用于验证调度器的重试链路。
    fail_first_goal_ = declare_parameter<bool>("fail_first_goal", false);
    // create_server 会注册三个回调：目标请求、取消请求和目标被接受后的执行入口。
    server_ = rclcpp_action::create_server<NavigateSequence>(
        this,
        "/navigate_sequence",
        std::bind(&MockNavigationServer::HandleGoal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&MockNavigationServer::HandleCancel, this,
                  std::placeholders::_1),
        std::bind(&MockNavigationServer::HandleAccepted, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "mock /navigate_sequence action server is ready");
  }

private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID&,
      const std::shared_ptr<const NavigateSequence::Goal> goal) {
    // 目标回调只做快速准入判断，不在这里执行耗时任务。
    if (goal->goals.empty()) {
      RCLCPP_WARN(get_logger(), "rejecting empty navigation sequence");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandle>) {
    // 教学实现无额外取消条件，统一接受客户端的取消请求。
    // 真正的导航系统可能需要检查当前控制器是否允许安全停止。
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    // Action 执行可能持续较长时间，因此放到独立线程，避免阻塞 ROS executor。
    // 这个 detached 线程只适合当前教学 Mock；生产代码应使用受控线程或 executor。
    std::thread{[this, goal_handle]() { 
      Execute(goal_handle); }
    }.detach();
  }

  void Execute(const std::shared_ptr<GoalHandle> goal_handle) {
    // get_goal() 返回客户端提交的只读 Goal；Server 不直接修改客户端对象。
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateSequence::Feedback>();

    if (fail_first_goal_ && !failure_injected_) {
      // 故障注入只发生一次，用来观察网关收到失败后是否重新入队并重试。
      failure_injected_ = true;
      auto result = std::make_shared<NavigateSequence::Result>();
      result->success = false;
      result->completed_goals = 0;
      result->message = "injected mock failure";
      goal_handle->abort(result);
      return;
    }

    // 外层循环模拟“逐个 waypoint 执行”，内层循环把每个 waypoint 拆成三段进度。
    // 真实导航会逐个到达目标；Mock 用循环代替真实位置控制器。
    for (std::size_t index = 0; index < goal->goals.size(); ++index) {
      for (int step = 1; step <= 3; ++step) {
        // 取消检查放在执行循环内，保证长任务不会等到全部目标完成才响应。
        if (goal_handle->is_canceling()) {
          // 取消必须通过 canceled() 结束当前 Goal，客户端才会收到 CANCELED 结果。
          auto result = std::make_shared<NavigateSequence::Result>();
          result->success = false;
          result->completed_goals = static_cast<std::uint32_t>(index);
          result->message = "navigation canceled";
          goal_handle->canceled(result);
          return;
        }

        // 每次循环发布一次反馈，模拟真实导航节点持续报告执行进度。
        feedback->current_goal_index = static_cast<std::uint32_t>(index);
        feedback->progress = static_cast<float>(
            (static_cast<double>(index) + static_cast<double>(step) / 3.0) /
            static_cast<double>(goal->goals.size()));
        feedback->state = "MOVING";
        goal_handle->publish_feedback(feedback);
        // 延时只用于制造可观察的异步过程；它不代表真实机器人运动时间。
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
    }

    // 所有目标完成后，调用 succeed() 发送最终结果并关闭 Action Goal。
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = true;
    result->completed_goals = static_cast<std::uint32_t>(goal->goals.size());
    result->message = "mock navigation completed";
    goal_handle->succeed(result);
  }

  // 必须保存 Server 的 SharedPtr，否则构造函数结束后服务会被销毁。
  rclcpp_action::Server<NavigateSequence>::SharedPtr server_;
  bool fail_first_goal_ = false;
  bool failure_injected_ = false;
};

}  // namespace learn_order_node

int main(int argc, char** argv) {
  // Mock Server 是独立 ROS2 进程，由 launch 文件与网关节点一起启动。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<learn_order_node::MockNavigationServer>());
  rclcpp::shutdown();
  return 0;
}
