#ifndef AGGREGATOR_CORE_HPP_
#define AGGREGATOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered_array.hpp"

namespace samples
{

/**
 * @class AggregatorCore
 * @brief 聚合器核心统计算法实现类，负责计算各话题的平均消息传输频率 (msg/s)
 */
class AggregatorCore
{
public:
  /**
   * @brief 构造函数，记录启动时的初始时间戳
   * @param timestamp 初始 Unix 时间戳（毫秒）
   */
  explicit AggregatorCore(int64_t timestamp);

  /**
   * @brief 计算 /unfiltered 话题的运行频率（条数 / 持续时长），若分母无效则返回 0.0
   * @return double 未滤波消息频率
   */
  double raw_frequency() const;

  /**
   * @brief 计算 /filtered 话题的运行频率，若分母无效则返回 0.0
   * @return double 滤波消息频率
   */
  double filtered_frequency() const;

  /**
   * @brief 记录接收到的未滤波原始消息，更新计数和最新时间戳
   * @param msg 原始消息指针
   */
  void add_raw_msg(
    const sample_msgs::msg::Unfiltered::SharedPtr msg);

  /**
   * @brief 记录接收到的滤波数组消息，遍历包内时间戳更新最新时间并增加计数
   * @param msg 滤波数组消息指针
   */
  void add_filtered_msg(
    const sample_msgs::msg::FilteredArray::SharedPtr msg);

private:
  // 接收到的未滤波与已滤波消息计数
  int raw_msg_count_;
  int filtered_msg_count_;

  // 节点初始启动时间戳（毫秒）
  int64_t start_;

  // 记录未滤波与已滤波消息的最近接收时间戳（毫秒）
  int64_t latest_raw_time_;
  int64_t latest_filtered_time_;
};

}  // namespace samples

#endif  // AGGREGATOR_CORE_HPP_

