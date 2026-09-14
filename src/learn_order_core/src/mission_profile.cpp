#include "learn_order_core/mission_profile.hpp"

#include <cmath>

namespace learn_order_core {
//验证任务模型本身是否满足最基本的输入契约。
bool ValidateMissionProfile(const MissionProfile& profile, std::string* message) {
  // 使用局部 lambda 统一填写失败消息，避免每个分支重复写相同样板代码。
  auto reject = [message](const std::string& reason) {
    if (message != nullptr) {
      *message = reason;
    }
    return false;
  };

  // 任务 ID 是后续去重、日志和状态查询的主键，不能为空。
  if (profile.mission_id.empty()) {
    return reject("order_id is empty");
  }
  // 没有坐标系就无法解释 waypoint 的位置。
  if (profile.frame_id.empty()) {
    return reject("frame_id is empty");
  }
  // 不同业务类型拥有不同的 waypoint 数量契约。
  // 运输任务必须包含 pickup + dropoff 两个点；巡检任务只需要一个检查点。
  const auto expected_waypoint_count =
      profile.kind == MissionKind::kTransport ? 2U : 1U;
  if (profile.waypoints.size() != expected_waypoint_count) {
    if (profile.kind == MissionKind::kTransport) {
      return reject("transport mission must contain pickup and dropoff waypoints");
    }
    return reject("inspection mission must contain one inspection waypoint");
  }
  // 防止 NaN/Inf 坐标进入距离计算和导航接口。
  for (const auto& waypoint : profile.waypoints) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
        !std::isfinite(waypoint.yaw)) {
      return reject("waypoint contains a non-finite value");
    }
  }
  if (message != nullptr) {
    *message = "ok";
  }
  return true;
}

}  // namespace learn_order_core
