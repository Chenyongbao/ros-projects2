#ifndef TRANSFORMER_NODE_HPP_
#define TRANSFORMER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/filtered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"
#include "sample_msgs/msg/unfiltered.hpp"

#include "transformer_core.hpp"

/**
 * @class TransformerNode
 * @brief ROS 2 转换器节点类：订阅未滤波数据，校验并提取三维坐标，达到指定容量后打包为 FilteredArray 批量发布
 */
class TransformerNode : public rclcpp::Node
{
public:
  // 发布订阅 QoS 历史深度
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * @brief 构造函数
   */
  TransformerNode();

private:
  /**
   * @brief 未滤波原始数据订阅回调函数
   * @param msg 接收到的原始字符串消息
   */
  void unfiltered_callback(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);

  // 原始数据订阅者 (/unfiltered_topic)
  rclcpp::Subscription<sample_msgs::msg::Unfiltered>::SharedPtr raw_sub_;

  // 批量滤波数据发布者 (/filtered_topic)
  rclcpp::Publisher<sample_msgs::msg::FilteredArray>::SharedPtr transform_pub_;

  // 内部转换与校验核心对象
  samples::TransformerCore transformer_;
};

#endif  // TRANSFORMER_NODE_HPP_

