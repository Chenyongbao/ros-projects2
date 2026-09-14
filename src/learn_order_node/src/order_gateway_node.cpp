#include "learn_order_core/mission_event_log.hpp"
#include "learn_order_core/mission_preflight.hpp"
#include "learn_order_core/mission_queue.hpp"
#include "learn_order_core/order_router.hpp"
#include "learn_order_interfaces/action/navigate_sequence.hpp"
#include "learn_order_interfaces/msg/mission_event.hpp"
#include "learn_order_interfaces/msg/mission_state.hpp"
#include "learn_order_interfaces/srv/cancel_mission.hpp"
#include "learn_order_interfaces/srv/get_mission_events.hpp"
#include "learn_order_interfaces/srv/get_mission_status.hpp"
#include "learn_order_interfaces/srv/submit_order.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace learn_order_node {

using NavigateSequence = learn_order_interfaces::action::NavigateSequence;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateSequence>;

class OrderGatewayNode final : public rclcpp::Node {
public:
  OrderGatewayNode() : Node("order_gateway_node") {
    // max_retries 表示同一个任务在导航失败后最多重新入队的次数。
    // 这里使用参数而不是常量，便于 launch 或命令行针对不同场景调整策略。
    max_retries_ = declare_parameter<int>("max_retries", 1);
    // 默认关闭自动派发，先保留手动 /dispatch_next 便于观察调度过程。
    auto_dispatch_ = declare_parameter<bool>("auto_dispatch", false);

    // Service：接收外部订单，一次请求对应一次接单响应。
    submit_service_ = create_service<learn_order_interfaces::srv::SubmitOrder>(
        "/v2/submit_order",
        std::bind(&OrderGatewayNode::HandleSubmit, this,
                  std::placeholders::_1, std::placeholders::_2));
    // Service：教学阶段手动触发一次队列派发。
    dispatch_service_ = create_service<std_srvs::srv::Trigger>(
        "/dispatch_next",
        std::bind(&OrderGatewayNode::HandleDispatch, this,
                  std::placeholders::_1, std::placeholders::_2));
    // Service：按任务 ID 取消；空 ID 保留旧行为，取消当前活动 Goal。
    cancel_service_ = create_service<learn_order_interfaces::srv::CancelMission>(
        "/cancel_mission",
        std::bind(&OrderGatewayNode::HandleCancel, this,
                  std::placeholders::_1, std::placeholders::_2));
    // Service：查询单个任务最新状态与是否终态。
    status_service_ = create_service<learn_order_interfaces::srv::GetMissionStatus>(
        "/mission_status",
        std::bind(&OrderGatewayNode::HandleStatus, this,
                  std::placeholders::_1, std::placeholders::_2));
    // 查询节点内存中保存的事件历史；空 mission_id 表示查询全部任务。
    event_query_service_ =
        create_service<learn_order_interfaces::srv::GetMissionEvents>(
            "/mission_events",
            std::bind(&OrderGatewayNode::HandleGetEvents, this,
                      std::placeholders::_1, std::placeholders::_2));

    state_publisher_ = create_publisher<std_msgs::msg::String>("/mission_state", 10);
    event_publisher_ = create_publisher<std_msgs::msg::String>("/mission_event", 10);
    // 结构化 Topic 是后续正式接口；字符串 Topic 暂时保留用于教学和兼容旧命令。
    structured_state_publisher_ =
        create_publisher<learn_order_interfaces::msg::MissionState>(
            "/mission_state_structured", 10);
    structured_event_publisher_ =
        create_publisher<learn_order_interfaces::msg::MissionEvent>(
            "/mission_event_structured", 10);
    navigate_client_ = rclcpp_action::create_client<NavigateSequence>(
        this, "/navigate_sequence");
    // 定时器固定周期检查队列，但只有打开 auto_dispatch 且 Action Server 已就绪时才派发。
    dispatch_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&OrderGatewayNode::HandleAutoDispatch, this));
    RCLCPP_INFO(get_logger(),
                "ready: /v2/submit_order, /dispatch_next, /cancel_mission and /mission_events");
  }

private:
  void HandleSubmit(
      const std::shared_ptr<learn_order_interfaces::srv::SubmitOrder::Request> request,
      std::shared_ptr<learn_order_interfaces::srv::SubmitOrder::Response> response) {
    // ROS2 生成的 Request 只属于通信层；先转换为纯 C++ 输入，后续核心代码不依赖 ROS。
    const learn_order_core::SubmitOrderInput input{
        request->order_id, request->order_type, request->priority,
        request->payload_json, request->tags};
    std::string message;

    // ROS 请求先转为核心输入，再由路由器生成统一 MissionProfile。
    const auto route = learn_order_core::BuildOrderRoute(input, &message);
    if (!route.has_value()) {
      // 路由失败意味着订单类型或 payload 无法解析，不能进入队列。
      Reject(response, "route", message);
      return;
    }
    // 同 ID 去重：只在“排队中”或“正在执行”两种进行中状态下拒绝。
    // 终态任务（SUCCEEDED / FAILED / CANCELED）允许重新提交同一 ID 的订单：
    // 终态后该 ID 已不在队列、也不再是活动任务，重复执行风险已经解除，
    // 重提会追加新的 QUEUED 事件，开启同一 ID 的新一轮生命周期。
    if (queue_.Contains(route->mission.mission_id) ||
        (mission_in_flight_ && active_mission_id_ == route->mission.mission_id)) {
      Reject(response, "duplicate",
             "mission already active or queued: " + route->mission.mission_id);
      return;
    }

    // 任务只有通过基础校验、电量和成本预检后才允许进入队列。
    const auto preflight = learn_order_core::ValidateMissionPreflight(
        route->mission, preflight_config_);
    if (!preflight.allowed) {
      // 预检失败时不改变调度器状态，调用者可以根据 message 修正订单。
      Reject(response, "preflight", preflight.message);
      return;
    }
    if (!queue_.Enqueue(route->mission, route->priority, &message)) {
      // 这里是最后一道入队保护；正常情况下重复任务已在上面被拦截。
      Reject(response, "queue", message);
      return;
    }

    // 新任务从零次重试开始；重试任务本身不会重新清零该计数。
    retry_counts_[route->mission.mission_id] = 0;
    response->accepted = true;
    response->reject_reason = "none";
    response->mission_id = route->mission.mission_id;
    response->queue_size = static_cast<int>(queue_.Size());
    response->message = message + "; " + preflight.message;
    RecordEvent(route->mission.mission_id, "QUEUED", response->message);
    PublishState("QUEUED", route->mission.mission_id);
  }

  void HandleDispatch(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Service 回调只负责把内部调度结果转换为 ROS 响应。
    std::string message;
    response->success = DispatchNext(&message);
    response->message = message;
  }

  bool DispatchNext(std::string* message) {
    // PopNext 会先把任务移出等待队列；后续任何派发失败路径都必须 Requeue。
    const auto result = queue_.PopNext();
    if (!result.success) {
      if (message != nullptr) *message = result.message;
      return false;
    }
    if (mission_in_flight_) {
      // 当前节点采用单机器人、单活动 Goal 模型，不允许并行导航。
      Requeue(result.mission);
      if (message != nullptr) *message = "another mission is already running";
      return false;
    }
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(1))) {
      // Action Server 可能因启动顺序尚未上线；短暂等待后把任务放回队列。
      Requeue(result.mission);
      if (message != nullptr) *message = "navigate_sequence action server is unavailable";
      return false;
    }

    // active_mission_ 保存完整任务，active_mission_id_ 用于日志/状态快速定位。
    const auto mission = result.mission;
    mission_in_flight_ = true;
    active_mission_id_ = mission.profile.mission_id;
    active_mission_ = mission;
    RecordEvent(active_mission_id_, "DISPATCHED", "mission sent to action server");
    PublishState("DISPATCHED", active_mission_id_);

    // 任务核心使用 MissionWaypoint，Action 使用 PoseStamped，这里完成边界适配。
    NavigateSequence::Goal goal;
    goal.goals.reserve(mission.profile.waypoints.size());
    for (const auto& waypoint : mission.profile.waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = mission.profile.frame_id;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      // 平面朝向 yaw 转四元数（绕 Z 轴旋转），保证到达朝向与订单一致；
      // 之前硬编码 w=1.0 会让所有任务的到达朝向变成 0 弧度。
      pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
      pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);
      goal.goals.push_back(std::move(pose));
    }

    // Action 的三个回调分别处理 Goal 接受、过程反馈和最终结果。
    rclcpp_action::Client<NavigateSequence>::SendGoalOptions options;
    options.goal_response_callback =
        [this, mission_id = active_mission_id_](
            const NavigateGoalHandle::SharedPtr& goal_handle) {
          if (!goal_handle) {
            HandleFailure(mission_id, "action goal rejected");
            return;
          }
          active_goal_handle_ = goal_handle;
          RecordEvent(mission_id, "RUNNING", "action goal accepted");
          PublishState("RUNNING", mission_id);
        };
    options.feedback_callback =
        [this, mission_id = active_mission_id_](
            NavigateGoalHandle::SharedPtr,
            const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "mission %s: waypoint=%u progress=%.2f state=%s",
              mission_id.c_str(), feedback->current_goal_index,
              static_cast<double>(feedback->progress), feedback->state.c_str());
        };
    options.result_callback =
        [this, mission_id = active_mission_id_](
            const NavigateGoalHandle::WrappedResult& wrapped_result) {
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              wrapped_result.result->success) {
            FinishActiveMission(mission_id, "SUCCEEDED", "mission completed", false);
          } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
            FinishActiveMission(mission_id, "CANCELED", "mission canceled", false);
          } else {
            HandleFailure(mission_id, "navigation action failed");
          }
        };

    // 非阻塞发送；后续状态变化由 ROS executor 调用上述回调完成。
    navigate_client_->async_send_goal(goal, options);
    if (message != nullptr) {
      *message = "mission dispatched to /navigate_sequence: " + mission.profile.mission_id;
    }
    return true;
  }
  //添加取消功能
  // 按 mission_id 取消任务，按任务当前位置分三条路径：
  //   1. mission_id 为空   -> 兼容 v1 旧行为：取消当前活动 Goal；
  //   2. 任务在排队中     -> 同步取消：移出队列并立即记录 CANCELED 终态；
  //   3. 任务正在执行     -> 异步取消：只发起 Goal 取消请求，
  //      最终 CANCELED 终态由 Action 的 result_callback 确认后才记录；
  //   4. 任务不存在       -> 失败，按事件历史给出 not_found / already_terminal。
  void HandleCancel(
      const std::shared_ptr<learn_order_interfaces::srv::CancelMission::Request> request,
      std::shared_ptr<learn_order_interfaces::srv::CancelMission::Response> response) {
    const auto& mission_id = request->mission_id;

    if (mission_id.empty()) {
      // 空 ID 保留旧行为：取消当前活动 Goal。
      // 没有活动任务时直接失败，不写任何事件（取消请求本身不产生任务状态变化）。
      if (!mission_in_flight_ || !active_goal_handle_) {
        response->success = false;
        response->reject_reason = "no_active_mission";
        response->message = "no active mission";
        return;
      }
      RequestActiveGoalCancel(response);
      return;
    }

    if (queue_.Contains(mission_id)) {
      // 排队任务尚未进入 Action，取消是同步、即时确定的：
      // 直接移出队列，并在本回调内就落 CANCELED 终态事件（is_terminal=true），
      // 调用方拿到 success=true 时任务一定已经结束，无需再查状态。
      std::string message;
      if (!queue_.Remove(mission_id, &message)) {
        // 理论上前面 Contains 已保证存在；防御性分支避免并发边界下误报成功。
        response->success = false;
        response->reject_reason = "not_in_queue";
        response->message = message;
        return;
      }
      // 带上该任务已有的重试次数，保证事件里的 retry_count 与历史一致。
      const int retry_count = retry_counts_[mission_id];
      RecordEvent(mission_id, "CANCELED", "canceled by operator while queued", retry_count);
      PublishState("CANCELED", mission_id);
      response->success = true;
      response->reject_reason = "none";
      response->message = "queued mission canceled: " + mission_id;
      return;
    }

    if (mission_in_flight_ && active_mission_id_ == mission_id) {
      // 在跑任务走异步 Goal 取消；这里不能直接写 CANCELED，
      // 最终状态必须等 Action result_callback 确认，避免“请求取消但仍在执行”的竞态。
      RequestActiveGoalCancel(response);
      return;
    }

    // 既不在队列也不在活动任务中：查事件历史给出更精确的失败原因。
    //   found=true  -> 任务已结束（already_terminal，附当前终态名）；
    //   found=false -> 从未见过该 ID（not_found）。
    const auto status = learn_order_core::ResolveMissionStatus(events_, mission_id);
    response->success = false;
    response->reject_reason = status.found ? "already_terminal" : "not_found";
    response->message = status.found
        ? "mission already in terminal state: " + mission_id + " (" + status.state + ")"
        : "mission not found: " + mission_id;
  }

  // 向当前活动 Goal 发起异步取消，并统一填好 Service 响应。
  // 单独抽出来的原因：空 ID（兼容旧行为）和“指定 ID 且任务正在执行”
  // 两条路径共用同样的取消动作，避免两处重复维护。
  void RequestActiveGoalCancel(
      std::shared_ptr<learn_order_interfaces::srv::CancelMission::Response> response) {
    // 先记录 CANCEL_REQUESTED 事件（过程状态，不是终态），
    // 这样事件时间线里能区分“请求取消”和“确认取消”两个时刻。
    RecordEvent(active_mission_id_, "CANCEL_REQUESTED", "cancel requested by operator");
    // 通过 GoalHandle 精确取消当前 Goal，而不是取消整个 Action Server 的所有任务。
    // async 发起后立即返回响应；Action Server 接受取消后，
    // result_callback 会以 CANCELED 结果回调，届时 FinishActiveMission 落终态。
    navigate_client_->async_cancel_goal(active_goal_handle_);
    // success=true 只表示“取消请求已受理”，不代表任务已经停止。
    response->success = true;
    response->reject_reason = "none";
    response->message = "mission cancellation requested: " + active_mission_id_;
  }

  // 查询单个任务的最新状态；解析逻辑由 core 的 ResolveMissionStatus 完成，
  // 这里只做 ROS 协议适配（core 结构体 -> Service 响应字段）。
  // 返回的 is_terminal 来自“最近一条事件”，因此终态后重提会正确变为 false。
  void HandleStatus(
      const std::shared_ptr<learn_order_interfaces::srv::GetMissionStatus::Request> request,
      std::shared_ptr<learn_order_interfaces::srv::GetMissionStatus::Response> response) {
    const auto status =
        learn_order_core::ResolveMissionStatus(events_, request->mission_id);
    response->found = status.found;
    response->state = status.state;
    response->is_terminal = status.is_terminal;
    response->retry_count = static_cast<std::uint32_t>(status.retry_count);
    // timestamp 只在终态时有意义（core 已保证非终态为 0），
    // 这里把 uint32 拆回 builtin_interfaces/Time 的 sec/nanosec 两个字段。
    response->timestamp.sec = status.terminal_sec;
    response->timestamp.nanosec = status.terminal_nanosec;
    response->message = status.found
        ? status.message
        : "no events found for mission: " + request->mission_id;
  }

  void HandleGetEvents(
      const std::shared_ptr<learn_order_interfaces::srv::GetMissionEvents::Request> request,
      std::shared_ptr<learn_order_interfaces::srv::GetMissionEvents::Response> response) {
    // ROS 回调只负责协议适配，查询筛选委托给不依赖 ROS 的核心函数。
    const auto matched = learn_order_core::FindMissionEvents(events_, request->mission_id);
    response->success = true;
    response->events.reserve(matched.size());
    for (const auto& event : matched) {
      std::ostringstream stream;
      stream << "mission_id=" << event.mission_id
             << ";state=" << event.state
             << ";retry_count=" << event.retry_count
             << ";message=" << event.message;
      response->events.push_back(stream.str());
    }
    if (matched.empty()) {
      response->message = request->mission_id.empty()
                              ? "no mission events recorded"
                              : "no events found for mission: " + request->mission_id;
    } else {
      response->message = "events found: " + std::to_string(matched.size());
    }
  }

  void HandleAutoDispatch() {
    // 单机器人模型只允许一个活动任务；Action Server 未上线时不调用会阻塞的等待函数。
    if (!auto_dispatch_ || mission_in_flight_ || queue_.Empty() ||
        !navigate_client_->action_server_is_ready()) {
      return;
    }
    std::string message;
    if (!DispatchNext(&message)) {
      RCLCPP_DEBUG(get_logger(), "automatic dispatch skipped: %s", message.c_str());
    }
  }

  void HandleFailure(const std::string& mission_id, const std::string& reason) {
    // 失败处理只针对 Action 的失败/拒绝结果；取消结果不会进入这里。
    const int retry_count = retry_counts_[mission_id];
    if (active_mission_.has_value() && retry_count < max_retries_) {
      const int next_retry = retry_count + 1;
      retry_counts_[mission_id] = next_retry;
      // 先复制活动任务，再清空活动状态，最后重新入队，保证状态变化顺序明确。
      const auto mission = *active_mission_;
      mission_in_flight_ = false;
      active_goal_handle_.reset();
      active_mission_.reset();
      active_mission_id_.clear();
      Requeue(mission);
      const auto message = reason + "; retry " + std::to_string(next_retry) + "/" +
                           std::to_string(max_retries_);
      RecordEvent(mission_id, "RETRY_QUEUED", message, next_retry);
      PublishState("RETRY_QUEUED", mission_id);
      // 为了保持闭环，重试任务入队后立即尝试再次派发。
      // 自动派发使一次订单无需人工再次调用 /dispatch_next 即可完成重试闭环。
      DispatchNext(nullptr);
      return;
    }
    FinishActiveMission(mission_id, "FAILED", reason, true);
  }

  void FinishActiveMission(
      const std::string& mission_id, const std::string& state,
      const std::string& message, const bool failed) {
    // 所有终态统一经过这里，集中完成事件记录、状态发布和活动任务清理。
    RecordEvent(mission_id, state, message, retry_counts_[mission_id]);
    PublishState(state, mission_id);
    if (failed) {
      RCLCPP_WARN(get_logger(), "mission %s failed: %s", mission_id.c_str(), message.c_str());
    }
    active_goal_handle_.reset();
    active_mission_.reset();
    active_mission_id_.clear();
    mission_in_flight_ = false;
  }

  void Requeue(const learn_order_core::QueuedMission& mission) {
    // 重新入队会由 MissionQueue 分配新的 sequence；优先级保持原任务优先级。
    std::string message;
    queue_.Enqueue(mission.profile, mission.priority, &message);
  }

  void Reject(
      const std::shared_ptr<learn_order_interfaces::srv::SubmitOrder::Response>& response,
      const std::string& reason, const std::string& message) {
    response->accepted = false;
    response->reject_reason = reason;
    response->queue_size = static_cast<int>(queue_.Size());
    response->message = message;
  }

  void PublishState(const std::string& state, const std::string& mission_id) {
    // 同时发布兼容字符串和结构化消息：前者便于教学观察，后者供正式消费者使用。
    // 字符串格式固定为 state / mission_id / queue_size 三段，
    // 下游旧代码若还在解析它，字段顺序不能随意调整。
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "state=" << state << ";mission_id=" << mission_id
           << ";queue_size=" << queue_.Size();
    message.data = stream.str();
    state_publisher_->publish(message);

    learn_order_interfaces::msg::MissionState structured;
    structured.state = state;
    // 终态标记统一用 core 的 IsTerminalState 判断，避免节点自己再维护一份字符串集合。
    structured.is_terminal = learn_order_core::IsTerminalState(state);
    // 发布时刻用节点时钟填充 timestamp，供下游做超时判断与事件排序。
    const rclcpp::Time current_time = this->now();
    structured.timestamp.sec = static_cast<std::int32_t>(current_time.seconds());
    structured.timestamp.nanosec = static_cast<std::uint32_t>(current_time.nanoseconds());
    structured.mission_id = mission_id;
    structured.queue_size = static_cast<std::uint32_t>(queue_.Size());
    structured.retry_count = mission_id.empty() ? 0U :
        static_cast<std::uint32_t>(retry_counts_[mission_id]);
    structured.message = message.data;
    structured_state_publisher_->publish(structured);
  }
  //记录事件
  // 统一的“事件落账”入口：任何任务状态变化（QUEUED / DISPATCHED / RUNNING /
  // RETRY_QUEUED / CANCEL_REQUESTED / SUCCEEDED / FAILED / CANCELED）
  // 都经过这里，向内部事件历史和 /mission_event 话题各记一笔，
  // 方便后续排查问题、事件追溯和状态查询（/mission_status 的数据来源）。
  // 注意：只有终态事件会同时触发 PublishState 之外的清理逻辑（见
  // FinishActiveMission），本函数本身不做任何调度状态变更。
  void RecordEvent(
      const std::string& mission_id, const std::string& state,
      const std::string& message, const int retry_count = 0) {
    // 先写入节点内存中的事件历史，再发布同一事件供命令行/监控订阅。
    const rclcpp::Time event_now = this->now();
    // 前 4 个字段按位置聚合初始化，后 3 个新字段按名赋值，
    // 这样既兼容 MissionEvent 的字段顺序契约，又保持可读性。
    learn_order_core::MissionEvent core_event{mission_id, state, message, retry_count};
    // 终态标记在"记账时"一次性算好，查询侧（ResolveMissionStatus）直接复用，
    // 避免同一状态在不同时刻被判断出不同结果。
    core_event.is_terminal = learn_order_core::IsTerminalState(state);
    // 事件时间戳取记账时刻的节点时钟，用于还原任务时间线与计算耗时。
    core_event.timestamp_sec = static_cast<std::uint32_t>(event_now.seconds());
    core_event.timestamp_nanosec = static_cast<std::uint32_t>(event_now.nanoseconds());
    learn_order_core::AppendMissionEvent(events_, core_event);
    std_msgs::msg::String event_message;
    std::ostringstream stream;
    stream << "mission_id=" << mission_id << ";state=" << state
           << ";retry_count=" << retry_count << ";message=" << message;
    event_message.data = stream.str();
    event_publisher_->publish(event_message);

    learn_order_interfaces::msg::MissionEvent structured;
    structured.mission_id = mission_id;
    structured.state = state;
    structured.is_terminal = core_event.is_terminal;
    structured.timestamp.sec = static_cast<std::int32_t>(event_now.seconds());
    structured.timestamp.nanosec = core_event.timestamp_nanosec;
    structured.retry_count = static_cast<std::uint32_t>(retry_count);
    structured.message = message;
    structured_event_publisher_->publish(structured);
  }

  // 等待执行的任务；任务被派发后从这里移除。
  learn_order_core::MissionQueue queue_;
  // 任务准入参数，决定坐标系、电量和成本估算规则。
  learn_order_core::PreflightConfig preflight_config_;
  // 当前进程内的有限事件历史；后续可接数据库或文件持久化。
  std::vector<learn_order_core::MissionEvent> events_;
  // 按 mission_id 保存已经发生的重试次数。
  std::unordered_map<std::string, int> retry_counts_;
  // 当前活动任务的完整快照，用于失败后重新入队。
  std::optional<learn_order_core::QueuedMission> active_mission_;
  int max_retries_ = 1;
  rclcpp::Service<learn_order_interfaces::srv::SubmitOrder>::SharedPtr submit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dispatch_service_;
  rclcpp::Service<learn_order_interfaces::srv::CancelMission>::SharedPtr cancel_service_;
  rclcpp::Service<learn_order_interfaces::srv::GetMissionStatus>::SharedPtr status_service_;
  rclcpp::Service<learn_order_interfaces::srv::GetMissionEvents>::SharedPtr
      event_query_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Publisher<learn_order_interfaces::msg::MissionState>::SharedPtr
      structured_state_publisher_;
  rclcpp::Publisher<learn_order_interfaces::msg::MissionEvent>::SharedPtr
      structured_event_publisher_;
  rclcpp_action::Client<NavigateSequence>::SharedPtr navigate_client_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
  NavigateGoalHandle::SharedPtr active_goal_handle_;
  std::string active_mission_id_;
  bool mission_in_flight_ = false;
  bool auto_dispatch_ = false;
};

}  // namespace learn_order_node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<learn_order_node::OrderGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
