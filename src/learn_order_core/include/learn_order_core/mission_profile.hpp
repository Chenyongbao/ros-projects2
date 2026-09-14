#pragma once

#include <string>
#include <vector>

namespace learn_order_core {

// 统一任务的业务类型。路由层负责把外部 order_type 转换为这个强类型枚举，
// 后续核心逻辑不再依赖字符串比较。
enum class MissionKind {
  kTransport,
  kInspection,
};

// 统一任务中的一个二维导航目标点。
// yaw 使用弧度，x/y/yaw 的含义由 MissionProfile::frame_id 决定。
struct MissionWaypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 可被后续导航执行器消费的统一任务模型。
// 外部订单无论来自 Service、REST 还是 VDA5050，最终都应转换成这个结构。
//ros2 内部真正执行任务模型结构体
struct MissionProfile {
  // 内部执行任务的唯一 ID。
  std::string mission_id;
  // 任务业务类型：运输任务包含 pickup/dropoff 两个点，巡检任务包含一个检查点。
  MissionKind kind = MissionKind::kTransport;
  // 所有 waypoint 所属坐标系；当前第一阶段要求为 map。
  std::string frame_id = "map";
  // 当前阶段没有使用循环任务，但保留字段便于后续扩展。
  std::vector<MissionWaypoint> waypoints;
};

// 检查任务是否满足执行器的输入契约，并通过 message 返回具体原因。
bool ValidateMissionProfile(const MissionProfile& profile, std::string* message);

}  // namespace learn_order_core
