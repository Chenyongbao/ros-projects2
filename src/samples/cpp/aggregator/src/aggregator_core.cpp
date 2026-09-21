// aggregator_core.cpp
// 聚合器核心统计算法实现：按时间戳差值统计各话题的平均发送频率

#include <algorithm>

#include "aggregator_core.hpp"

namespace samples
{

/**
 * @brief 构造函数，记录启动起始时间戳并初始化计数
 */
AggregatorCore::AggregatorCore(int64_t timestamp)
: raw_msg_count_(0), filtered_msg_count_(0), start_(timestamp),
  latest_raw_time_(-1), latest_filtered_time_(-1)
{}

/**
 * @brief 计算未滤波消息的接收频率（消息条数 / 经过的时间秒数）
 */
double AggregatorCore::raw_frequency() const
{
  // 检查是否具备有效的时间跨度（避免除以零或负数）
  if (latest_raw_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(raw_msg_count_) / (latest_raw_time_ - start_);
}

/**
 * @brief 计算滤波消息数组的接收频率（数组批次数 / 经过的时间秒数）
 */
double AggregatorCore::filtered_frequency() const
{
  if (latest_filtered_time_ <= start_) {
    return 0.0;
  }
  return static_cast<double>(filtered_msg_count_) / (latest_filtered_time_ - start_);
}

/**
 * @brief 记录一条未滤波消息，更新最大时间戳并累加总消息数
 */
void AggregatorCore::add_raw_msg(
  const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  latest_raw_time_ = std::max(
    static_cast<int64_t>(msg->timestamp), latest_raw_time_);
  raw_msg_count_++;
}

/**
 * @brief 记录一批滤波数组消息，提取包内最新时间戳并累加批次数
 */
void AggregatorCore::add_filtered_msg(
  const sample_msgs::msg::FilteredArray::SharedPtr msg)
{
  for (auto filtered_msg : msg->packets) {
    latest_filtered_time_ = std::max(
      static_cast<int64_t>(filtered_msg.timestamp), latest_filtered_time_);
  }
  filtered_msg_count_++;
}

}  // namespace samples

