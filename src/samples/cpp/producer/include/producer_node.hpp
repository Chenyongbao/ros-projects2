#ifndef PRODUCER_NODE_HPP_
#define PRODUCER_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sample_msgs/msg/unfiltered.hpp"

#include "producer_core.hpp"

/**
 * @class ProducerNode
 * @brief ROS 2 生产者节点类：按照设定的时间间隔生成未滤波的消息并发布到 /unfiltered_topic
 */
class ProducerNode : public rclcpp::Node
{
public:
  // 发布队列缓存深度：保留最近 20 条消息
  static constexpr int ADVERTISING_FREQ = 20;

  /**
   * @brief 构造函数
   * @param delay_ms 产生数据的定时器周期（毫秒）
   */
  explicit ProducerNode(int delay_ms);

private:
  /**
   * @brief 定时器回调函数：触发数据累加更新并将结果发布至话题
   */
  void timer_callback();

  /**
   * @brief 动态参数修改回调函数（运行时动态修改 velocity 速度参数）
   * @param parameters 被修改的参数列表
   * @return rcl_interfaces::msg::SetParametersResult 修改结果状态
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  // ROS 2 消息发布者，向 /unfiltered_topic 发布原始数据
  rclcpp::Publisher<sample_msgs::msg::Unfiltered>::SharedPtr data_pub_;

  // 定时器，按固定间隔调用数据生成回调
  rclcpp::TimerBase::SharedPtr timer_;

  // 动态参数回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // 生产者核心算法逻辑对象
  samples::ProducerCore producer_;
};

#endif  // PRODUCER_NODE_HPP_

