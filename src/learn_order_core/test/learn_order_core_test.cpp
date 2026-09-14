#include "learn_order_core/mission_event_log.hpp"
#include "learn_order_core/mission_preflight.hpp"
#include "learn_order_core/mission_queue.hpp"
#include "learn_order_core/order_router.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace learn_order_core {
namespace {

// 构造一份所有测试都可复用的合法坐标订单输入。
SubmitOrderInput ValidInput(const std::string& id = "order_001", int priority = 0) {
  return SubmitOrderInput{
      id,
      "transport",
      priority,
      R"({"frame_id":"map","pickup_x":1.0,"pickup_y":2.0,"pickup_yaw":0.0,"dropoff_x":5.0,"dropoff_y":3.0,"dropoff_yaw":1.57})",
      {"test"}};
}

// 构造一份合法的巡检订单：它只描述一个需要检查的目标点。
SubmitOrderInput ValidInspectionInput(const std::string& id = "inspection_001",
                                       int priority = 0) {
  return SubmitOrderInput{
      id,
      "inspection",
      priority,
      R"({"frame_id":"map","x":8.0,"y":4.0,"yaw":0.0})",
      {"test"}};
}

TEST(OrderRouterTest, RoutesTransportAndNormalizesType) {
  // 验证外部类型的格式差异不会影响内部路由。
  auto input = ValidInput();
  input.order_type = "Transport-Order";
  std::string message;
  const auto route = BuildOrderRoute(input, &message);

  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->normalized_order_type, "transport");
  EXPECT_EQ(route->mission.waypoints.size(), 2U);
}

TEST(OrderRouterTest, RejectsUnknownType) {
  // 第一阶段明确拒绝尚未实现的业务类型。
  auto input = ValidInput();
  input.order_type = "station_transport";
  std::string message;

  EXPECT_FALSE(BuildOrderRoute(input, &message).has_value());
}

TEST(OrderRouterTest, RejectsInvalidPayload) {
  // payload 缺少坐标字段时不能生成可执行任务。
  auto input = ValidInput();
  input.payload_json = "{}";
  std::string message;

  EXPECT_FALSE(BuildOrderRoute(input, &message).has_value());
}

TEST(OrderRouterTest, RoutesInspectionToSingleWaypoint) {
  // 巡检订单使用单点 payload，并且在内部被标记为 inspection 类型。
  std::string message;
  const auto route = BuildOrderRoute(ValidInspectionInput(), &message);

  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->kind, OrderKind::kInspection);
  EXPECT_EQ(route->mission.kind, MissionKind::kInspection);
  EXPECT_EQ(route->mission.waypoints.size(), 1U);
  EXPECT_EQ(message, "inspection order routed");
}

TEST(OrderRouterTest, RejectsInvalidInspectionPayload) {
  // 巡检订单不能借用 transport 的 pickup/dropoff 字段，必须提供 x/y/yaw。
  auto input = ValidInspectionInput();
  input.payload_json = R"({"frame_id":"map","x":8.0,"y":4.0})";
  std::string message;

  EXPECT_FALSE(BuildOrderRoute(input, &message).has_value());
}

TEST(PreflightTest, CalculatesCost) {
  // 从 (1,2) 到 (5,3) 的距离为 sqrt(17)。
  std::string message;
  const auto route = BuildOrderRoute(ValidInput(), &message);
  ASSERT_TRUE(route.has_value());

  const auto result = ValidateMissionPreflight(route->mission, PreflightConfig{});
  EXPECT_TRUE(result.allowed);
  EXPECT_NEAR(result.cost.distance_m, std::sqrt(17.0), 1e-9);
}

TEST(PreflightTest, AcceptsInspectionWithOneWaypoint) {
  // 巡检任务的单点结构应通过 profile 校验和 preflight，且当前成本为 0 米。
  std::string message;
  const auto route = BuildOrderRoute(ValidInspectionInput(), &message);
  ASSERT_TRUE(route.has_value());

  const auto result = ValidateMissionPreflight(route->mission, PreflightConfig{});
  EXPECT_TRUE(result.allowed);
  EXPECT_DOUBLE_EQ(result.cost.distance_m, 0.0);
  EXPECT_DOUBLE_EQ(result.cost.eta_s, 0.0);
}

TEST(QueueTest, UsesPriorityAndStableArrivalOrder) {
  // 20 优先级先于 10；两个 10 按入队顺序 first、third。
  MissionQueue queue;
  std::string message;
  const auto first = BuildOrderRoute(ValidInput("first", 10), &message);
  const auto second = BuildOrderRoute(ValidInput("second", 20), &message);
  const auto third = BuildOrderRoute(ValidInput("third", 10), &message);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(third.has_value());

  ASSERT_TRUE(queue.Enqueue(first->mission, 10, &message));
  ASSERT_TRUE(queue.Enqueue(second->mission, 20, &message));
  ASSERT_TRUE(queue.Enqueue(third->mission, 10, &message));

  EXPECT_EQ(queue.PopNext().mission.profile.mission_id, "second");
  EXPECT_EQ(queue.PopNext().mission.profile.mission_id, "first");
  EXPECT_EQ(queue.PopNext().mission.profile.mission_id, "third");
}

TEST(QueueTest, RejectsDuplicateMission) {
  // 同一个任务 ID 不允许重复进入等待队列。
  MissionQueue queue;
  std::string message;
  const auto route = BuildOrderRoute(ValidInput(), &message);
  ASSERT_TRUE(route.has_value());

  EXPECT_TRUE(queue.Enqueue(route->mission, 1, &message));
  EXPECT_FALSE(queue.Enqueue(route->mission, 1, &message));
}

TEST(QueueTest, AcceptsInspectionMission) {
  // inspection 与 transport 共用同一个队列，不需要为每种业务建立一套队列实现。
  MissionQueue queue;
  std::string message;
  const auto route = BuildOrderRoute(ValidInspectionInput("inspection_queue", 7), &message);
  ASSERT_TRUE(route.has_value());
  ASSERT_TRUE(queue.Enqueue(route->mission, route->priority, &message));

  const auto result = queue.PopNext();
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.mission.profile.mission_id, "inspection_queue");
  EXPECT_EQ(result.mission.profile.kind, MissionKind::kInspection);
  EXPECT_EQ(result.mission.profile.waypoints.size(), 1U);
}

TEST(MissionEventLogTest, FindsEventsByMissionId) {
  std::vector<MissionEvent> events;
  AppendMissionEvent(events, MissionEvent{"order_a", "QUEUED", "queued", 0});
  AppendMissionEvent(events, MissionEvent{"order_b", "QUEUED", "queued", 0});
  AppendMissionEvent(events, MissionEvent{"order_a", "SUCCEEDED", "done", 0});

  const auto matched = FindMissionEvents(events, "order_a");
  ASSERT_EQ(matched.size(), 2U);
  EXPECT_EQ(matched[0].state, "QUEUED");
  EXPECT_EQ(matched[1].state, "SUCCEEDED");
}

TEST(MissionEventLogTest, EmptyIdReturnsAllEvents) {
  std::vector<MissionEvent> events;
  AppendMissionEvent(events, MissionEvent{"order_a", "QUEUED", "queued", 0});
  AppendMissionEvent(events, MissionEvent{"order_b", "FAILED", "failed", 1});

  EXPECT_EQ(FindMissionEvents(events, "").size(), 2U);
}

TEST(MissionEventLogTest, TerminalStateRecognition) {
  // 锁死终态集合的边界：恰好是“成功 / 失败 / 取消”三种。
  // 任何过程状态都不算终态——尤其是 CANCEL_REQUESTED（只发了取消请求，
  // 任务可能仍在执行）和 RETRY_QUEUED（还在排队等待重试），
  // 把这两者误判成终态会导致下游提前认为任务已结束。
  EXPECT_TRUE(IsTerminalState("SUCCEEDED"));
  EXPECT_TRUE(IsTerminalState("FAILED"));
  EXPECT_TRUE(IsTerminalState("CANCELED"));
  EXPECT_FALSE(IsTerminalState("QUEUED"));
  EXPECT_FALSE(IsTerminalState("DISPATCHED"));
  EXPECT_FALSE(IsTerminalState("RUNNING"));
  EXPECT_FALSE(IsTerminalState("RETRY_QUEUED"));
  EXPECT_FALSE(IsTerminalState("CANCEL_REQUESTED"));
}

TEST(MissionEventLogTest, ResolvesLatestStatusPerMission) {
  // 验证“最新状态”的解析规则：按任务 ID 各取各的最后一条事件，
  // 不同任务的事件相互独立，不会串扰。
  //
  // 事件时间线（交错发生）：
  //   order_a -> QUEUED      （排队）
  //   order_b -> QUEUED      （排队）
  //   order_a -> SUCCEEDED   （终态，带 100 秒 200 纳秒的时间戳）
  std::vector<MissionEvent> events;
  AppendMissionEvent(events, MissionEvent{"order_a", "QUEUED", "queued", 0});
  AppendMissionEvent(events, MissionEvent{"order_b", "QUEUED", "queued", 0});
  // 7 字段按位置初始化：id / state / message / retry_count / is_terminal / sec / nanosec。
  // 字段顺序契约见 MissionEvent 注释（新增字段必须追加在末尾）。
  MissionEvent terminal{"order_a", "SUCCEEDED", "done", 0, true, 100, 200};
  AppendMissionEvent(events, terminal);

  // order_a 的最新事件是终态 SUCCEEDED，终态时刻应原样带出。
  const auto status_a = ResolveMissionStatus(events, "order_a");
  ASSERT_TRUE(status_a.found);
  EXPECT_EQ(status_a.state, "SUCCEEDED");
  EXPECT_TRUE(status_a.is_terminal);
  EXPECT_EQ(status_a.terminal_sec, 100U);
  EXPECT_EQ(status_a.terminal_nanosec, 200U);

  // order_b 从未终态：is_terminal 为 false，terminal_sec 必须保持 0，
  // 不能把别的任务的终态时刻误挂过来。
  const auto status_b = ResolveMissionStatus(events, "order_b");
  ASSERT_TRUE(status_b.found);
  EXPECT_EQ(status_b.state, "QUEUED");
  EXPECT_FALSE(status_b.is_terminal);
  EXPECT_EQ(status_b.terminal_sec, 0U);

  // 从未出现过的事件返回 found=false，且 state 为空（不是残留值）。
  const auto missing = ResolveMissionStatus(events, "order_c");
  EXPECT_FALSE(missing.found);
  EXPECT_TRUE(missing.state.empty());
}

TEST(MissionEventLogTest, ResolvesNonTerminalAfterTerminal) {
  // 验证“终态后允许重提”的解析语义：
  // 任务先被取消（CANCELED 终态），之后同一 ID 重新提交订单（QUEUED），
  // 此时“最新状态”必须回到过程状态 QUEUED，而不是停留在旧终态——
  // 否则下游会误以为重提后的新任务已经结束。
  //
  // 事件时间线：
  //   order_a -> CANCELED   （终态，10 秒）
  //   order_a -> QUEUED     （重提后的新生命周期开始）
  std::vector<MissionEvent> events;
  AppendMissionEvent(events, MissionEvent{"order_a", "CANCELED", "canceled", 0, true, 10, 0});
  AppendMissionEvent(events, MissionEvent{"order_a", "QUEUED", "resubmitted", 0});

  // 当前视角：最新事件是 QUEUED，is_terminal 必须为 false。
  const auto status = ResolveMissionStatus(events, "order_a");
  ASSERT_TRUE(status.found);
  EXPECT_EQ(status.state, "QUEUED");
  EXPECT_FALSE(status.is_terminal);
  // 历史视角：HasTerminalState 仍记得旧终态，供节点做
  // “同 ID 上一轮生命周期已结束”的判断；order_b 无任何事件，必须为 false。
  EXPECT_TRUE(HasTerminalState(events, "order_a"));
  EXPECT_FALSE(HasTerminalState(events, "order_b"));
}

TEST(QueueTest, RemovesQueuedMission) {
  // 验证“取消排队任务”的队列侧行为：
  //   1. 按 ID 移除成功：任务不再在队列，且不影响剩余任务的出队顺序；
  //   2. 移除不存在的 ID：返回 false，队列内容不变（幂等的安全操作）。
  MissionQueue queue;
  std::string message;
  // 两个不同 ID 的任务：first 优先级 1，second 优先级 2。
  const auto first = BuildOrderRoute(ValidInput("first", 1), &message);
  const auto second = BuildOrderRoute(ValidInput("second", 2), &message);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(queue.Enqueue(first->mission, 1, &message));
  ASSERT_TRUE(queue.Enqueue(second->mission, 2, &message));

  // 移除 first 后：Contains 查不到、队列只剩 1 个；
  // 出队的应是 second（它本来优先级就高，移除也不改变这一点）。
  EXPECT_TRUE(queue.Remove("first", &message));
  EXPECT_FALSE(queue.Contains("first"));
  EXPECT_EQ(queue.Size(), 1U);
  EXPECT_EQ(queue.PopNext().mission.profile.mission_id, "second");

  // 对已经移除（或从未存在）的 ID 再次 Remove 必须返回 false，
  // 且不能让队列大小变成负数或丢失其他任务。
  EXPECT_FALSE(queue.Remove("first", &message));
  EXPECT_EQ(queue.Size(), 0U);
}

}  // namespace
}  // namespace learn_order_core
