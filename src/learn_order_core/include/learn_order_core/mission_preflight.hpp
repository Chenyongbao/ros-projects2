#pragma once

#include <string>

#include "learn_order_core/mission_profile.hpp"

namespace learn_order_core {

// 任务准入前计算出的估算结果。
struct MissionCostEstimate {
  // 本次任务用于估算的平面距离，单位米。
  double distance_m = 0.0;
  // 预计执行时间，单位秒。
  double eta_s = 0.0;
  // 预计电池电压下降，单位伏特。
  double battery_drop = 0.0;
  // 执行任务结束后的预计电压。
  double projected_battery = 0.0;
};

// 预检所需的运行参数。真实项目中通常来自 ROS 参数或配置文件。
struct PreflightConfig {
  std::string expected_frame = "map";
  double nominal_speed_mps = 0.35;
  double current_battery = 24.0;
  double battery_drop_per_meter = 0.015;
  double minimum_battery = 22.0;
};

// 预检结果：allowed 决定任务是否可以进入队列。
struct PreflightResult {
  bool allowed = false;
  std::string message;
  MissionCostEstimate cost;
};

// 先校验任务，再估算距离/ETA/电量，最后给出是否允许入队的结论。
PreflightResult ValidateMissionPreflight(
    const MissionProfile& profile, const PreflightConfig& config);

}  // namespace learn_order_core
