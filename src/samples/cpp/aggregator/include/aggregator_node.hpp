#ifndef AGGREGATOR_NODE_HPP_
#define AGGREGATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"

#include "aggregator_core.hpp"

/**
 * @class AggregatorNode
 * @brief ROS 2 聚合器节点类：同时监听未滤波与滤波话题，实时向控制台终端输出两个话题的运行频率
 */
class AggregatorNode : public rclcpp::Node
{
public:
  // 订阅历史深度配置
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * @brief 构造函数
   */
  AggregatorNode();

private:
  /**
   * @brief 未滤波原始数据订阅回调函数
   * @param msg 原始消息指针
   */
  void unfiltered_callback(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);

  /**
   * @brief 滤波数组数据订阅回调函数
   * @param msg 滤波数组消息指针
   */
  void filtered_callback(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

  // 原始数据订阅者 (/unfiltered_topic)
  rclcpp::Subscription<sample_msgs::msg::Unfiltered>::SharedPtr raw_sub_;

  // 滤波数组数据订阅者 (/filtered_topic)
  rclcpp::Subscription<sample_msgs::msg::FilteredArray>::SharedPtr filtered_sub_;

  // 频率统计核心算法对象
  samples::AggregatorCore aggregator_;
};

#endif  // AGGREGATOR_NODE_HPP_

