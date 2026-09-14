#pragma once

#include <string>
#include <vector>

namespace learn_order_core {

// ROS Service 请求转换后的内部输入。
// 核心库只依赖标准 C++ 类型，因此不需要链接 rclcpp，也可以单独做单元测试。
struct SubmitOrderInput {
  // 外部订单的唯一标识，后续也作为 MissionProfile 的 ID。
  std::string order_id;
  // 原始订单类型，进入路由器后会被清洗为统一格式。
  std::string order_type;
  // 任务优先级，最终写入队列元素。
  int priority = 0;
  // 不同业务类型的可变参数；第一阶段解析坐标字段。
  std::string payload_json;
  // 外部来源标签，例如 cli、rest 或 wms。
  std::vector<std::string> tags;
};

}  // namespace learn_order_core
