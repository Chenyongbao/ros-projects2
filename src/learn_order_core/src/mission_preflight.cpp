#include "learn_order_core/mission_preflight.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace learn_order_core {

PreflightResult ValidateMissionPreflight(
    const MissionProfile& profile, const PreflightConfig& config) {
  PreflightResult result;
  std::string validation_message;
  // 预检的第一步是复用统一任务模型的基础校验。
  if (!ValidateMissionProfile(profile, &validation_message)) {
    result.message = validation_message;
    return result;
  }
  // 当前地图和后续导航只接受约定的坐标系，暂不做 TF 转换。
  if (profile.frame_id != config.expected_frame) {
    result.message = "unsupported frame_id: " + profile.frame_id;
    return result;
  }

  // 这里使用直线距离作为成本估算；后续可替换为 Nav2 ComputePathToPose 的真实路径长度。
  // 运输任务计算 pickup 到 dropoff 的距离；巡检任务只有一个目标点，
  // 当前没有起始位姿输入，因此把单点任务的估算距离设为 0 米。
  if (profile.kind == MissionKind::kTransport) {
    const auto& start = profile.waypoints[0];
    const auto& goal = profile.waypoints[1];
    result.cost.distance_m = std::hypot(goal.x - start.x, goal.y - start.y);
  } else {
    result.cost.distance_m = 0.0;
  }
  // 速度和单位距离消耗做下限保护，避免除零或负电量消耗。
  result.cost.eta_s = result.cost.distance_m / std::max(config.nominal_speed_mps, 0.01);
  result.cost.battery_drop =
      result.cost.distance_m * std::max(config.battery_drop_per_meter, 0.0);
  result.cost.projected_battery = config.current_battery - result.cost.battery_drop;
  // 预计结束电量低于安全阈值时，订单在入队前拒绝。
  if (result.cost.projected_battery < config.minimum_battery) {
    std::ostringstream stream;
    stream << "battery insufficient: projected " << result.cost.projected_battery
           << "V below minimum " << config.minimum_battery << "V";
    result.message = stream.str();
    return result;
  }

  // 所有检查通过，任务才有资格进入调度队列。
  result.allowed = true;
  std::ostringstream stream;
  stream << "preflight ok: " << result.cost.distance_m << "m, eta " << result.cost.eta_s
         << "s";
  result.message = stream.str();
  return result;
}

}  // namespace learn_order_core
