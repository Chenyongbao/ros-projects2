// aggregator_node.cpp
// ROS 2 聚合器节点实现：订阅两类数据并在控制台打印实时计算出的接收频率

#include <chrono>
#include <memory>

#include "aggregator_node.hpp"

/**
 * @brief 构造函数：记录当前系统时间戳初始化核心对象，并创建两个话题订阅者
 */
AggregatorNode::AggregatorNode()
: Node("aggregator"),
  aggregator_(
    samples::AggregatorCore(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()))
{
  // 订阅未滤波原始数据
  raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "/unfiltered_topic", ADVERTISING_FREQ,
    std::bind(
      &AggregatorNode::unfiltered_callback, this,
      std::placeholders::_1));

  // 订阅批量滤波数据数组
  filtered_sub_ = this->create_subscription<sample_msgs::msg::FilteredArray>(
    "/filtered_topic", ADVERTISING_FREQ,
    std::bind(
      &AggregatorNode::filtered_callback, this,
      std::placeholders::_1));
}

/**
 * @brief 未滤波数据接收回调：更新统计并打印未滤波话题频率 (msg/s)
 */
void AggregatorNode::unfiltered_callback(
  const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  aggregator_.add_raw_msg(msg);
  RCLCPP_INFO(
    this->get_logger(), "原始数据传输频率 (条/秒): %f",
    aggregator_.raw_frequency() * 1000);
}

/**
 * @brief 滤波数据数组接收回调：更新统计并打印滤波话题频率 (msg/s)
 */
void AggregatorNode::filtered_callback(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  aggregator_.add_filtered_msg(msg);
  RCLCPP_INFO(
    this->get_logger(), "滤波后数据传输频率 (条/秒): %f",
    aggregator_.filtered_frequency() * 1000);
}

/**
 * @brief 节点入口主函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AggregatorNode>());
  rclcpp::shutdown();
  return 0;
}

