#pragma once

#include <optional>       //可能存在/不存在
#include <string>

#include "learn_order_core/mission_profile.hpp"
#include "learn_order_core/order_input.hpp"

namespace learn_order_core {

// 当前学习阶段支持运输和巡检两种业务分支；后续可扩展 station、fleet、vda5050 等类型。
enum class OrderKind {
  kTransport,
  // 教学用模拟业务：机器人前往一个点执行一次检查。
  kInspection,
};

// 外部弱类型请求被路由后形成的内部结果。
//原始订单已经完成路由和解析
struct RoutedOrder {
  // 判别后的业务类别。
  OrderKind kind = OrderKind::kTransport;
  // 清洗后的订单类型，供日志和后续分支使用。
  std::string normalized_order_type;
  // 从 Service 请求继承的优先级。
  int priority = 0;
  // 已经解析完 payload 并生成 waypoint 的统一任务。
  MissionProfile mission;
};

// 将大小写、连字符、斜杠等输入归一化为小写下划线格式。
std::string NormalizeOrderType(const std::string& value);

// 解析订单类型和 payload_json，成功返回 RoutedOrder，失败返回 nullopt。
std::optional<RoutedOrder> BuildOrderRoute(
    const SubmitOrderInput& input, std::string* message);

}  // namespace learn_order_core
