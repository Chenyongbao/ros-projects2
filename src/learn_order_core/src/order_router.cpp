#include "learn_order_core/order_router.hpp"

#include <algorithm>
#include <cctype>
#include <regex>

namespace learn_order_core {
namespace {

// 去除订单类型首尾空格，避免用户输入 " transport " 导致路由失败。
std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

bool ReadString(const std::string& json, const std::string& key, std::string* value) {
  // 学习项目只解析固定的扁平 JSON 字段；生产项目应使用正式 JSON 库。
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) return false;
  *value = match[1].str();
  return true;
}

bool ReadDouble(const std::string& json, const std::string& key, double* value) {
  // 从 payload 中读取坐标数字，并拒绝无法转换的文本。
  const std::regex pattern(
      "\\\"" + key + "\\\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern)) return false;
  try {
    *value = std::stod(match[1].str());
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

std::string NormalizeOrderType(const std::string& value) {
  // 将 Transport-Order、TRANSPORT_ORDER 等写法收敛为可比较的标准字符串。
  std::string result;
  result.reserve(value.size());
  for (const auto ch : Trim(value)) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) != 0) {
      result.push_back(static_cast<char>(std::tolower(uch)));
    } else if (ch == '-' || ch == '_' || ch == '/') {
      result.push_back('_');
    }
  }
  return result;
}

std::optional<RoutedOrder> BuildOrderRoute(
    const SubmitOrderInput& input, std::string* message) {
  // 统一的路由入口：先判断订单类型，再解析该类型需要的业务字段。
  auto reject = [message](const std::string& reason) -> std::optional<RoutedOrder> {
    if (message != nullptr) *message = reason;
    return std::nullopt;
  };
  //归一化订单类型
  const auto type = NormalizeOrderType(input.order_type);
  if (type != "transport" && type != "transport_order" &&
      type != "inspection" && type != "inspection_order" &&
      type != "inspect") {
    return reject("unsupported order_type: " + type);
  }
  if (input.order_id.empty()) {
    return reject("order_id is empty");
  }
  if (input.payload_json.empty()) {
    return reject("payload_json is empty");
  }

  // 路由结果从这里开始填充，后续节点不再直接接触原始 payload 文本。
  //外部传进来的原始业务若JSON参数转换成机器可读的数据
  RoutedOrder route;
  route.normalized_order_type =
      (type == "transport" || type == "transport_order") ? "transport" : "inspection";
  route.kind = route.normalized_order_type == "transport"
                   ? OrderKind::kTransport
                   : OrderKind::kInspection;
  route.mission.kind = route.kind == OrderKind::kTransport
                           ? MissionKind::kTransport
                           : MissionKind::kInspection;
  route.priority = input.priority;
  route.mission.mission_id = input.order_id;

  // frame_id 省略时使用 map 默认坐标系。
  if (!ReadString(input.payload_json, "frame_id", &route.mission.frame_id)) {
    route.mission.frame_id = "map";
  }
  if (route.kind == OrderKind::kTransport) {
    // transport 订单必须同时提供 pickup 和 dropoff 两个完整位姿。
    const bool parsed =
        ReadDouble(input.payload_json, "pickup_x", &route.mission.waypoints.emplace_back().x);
    if (!parsed) {
      route.mission.waypoints.clear();
      return reject("payload_json missing or invalid pickup_x");
    }
    route.mission.waypoints.back().y = 0.0;
    if (!ReadDouble(input.payload_json, "pickup_y", &route.mission.waypoints.back().y) ||
        !ReadDouble(input.payload_json, "pickup_yaw", &route.mission.waypoints.back().yaw)) {
      route.mission.waypoints.clear();
      return reject("payload_json missing or invalid pickup waypoint");
    }

    route.mission.waypoints.emplace_back();
    if (!ReadDouble(input.payload_json, "dropoff_x", &route.mission.waypoints.back().x) ||
        !ReadDouble(input.payload_json, "dropoff_y", &route.mission.waypoints.back().y) ||
        !ReadDouble(input.payload_json, "dropoff_yaw", &route.mission.waypoints.back().yaw)) {
      route.mission.waypoints.clear();
      return reject("payload_json missing or invalid dropoff waypoint");
    }
  } else {
    // inspection 是最小模拟业务，只需要一个目标点。
    route.mission.waypoints.emplace_back();
    if (!ReadDouble(input.payload_json, "x", &route.mission.waypoints.back().x) ||
        !ReadDouble(input.payload_json, "y", &route.mission.waypoints.back().y) ||
        !ReadDouble(input.payload_json, "yaw", &route.mission.waypoints.back().yaw)) {
      route.mission.waypoints.clear();
      return reject("payload_json missing or invalid inspection waypoint");
    }
  }

  if (message != nullptr) {
    *message = route.kind == OrderKind::kTransport
                   ? "transport order routed"
                   : "inspection order routed";
  }
  return route;
}

}  // namespace learn_order_core
