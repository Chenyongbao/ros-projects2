// aggregator_test.cpp
// 聚合器核心统计算法单元测试：验证除零保护、单条/多条消息到达以及乱序时间戳下的频率计算正确性

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "aggregator_core.hpp"

// 测试用例 1：测试当时间戳与起始时间相同时的除零保护（未滤波话题）
TEST(AggregatorTest, RawDivisionByZero)
{
  samples::AggregatorCore aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();
  msg->timestamp = 0;

  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.0, aggregator.raw_frequency());
}

// 测试用例 2：测试当时间戳与起始时间相同时的除零保护（滤波话题）
TEST(AggregatorTest, FilteredDivisionByZero)
{
  samples::AggregatorCore aggregator(1);
  auto filtered_packet = std::make_shared<sample_msgs::msg::FilteredArray>();
  std::vector<sample_msgs::msg::Filtered> msgs;
  auto msg = sample_msgs::msg::Filtered();
  msg.timestamp = 1;
  msgs.push_back(msg);
  filtered_packet->packets = msgs;

  aggregator.add_filtered_msg(filtered_packet);
  EXPECT_DOUBLE_EQ(0.0, aggregator.filtered_frequency());
}

// 测试用例 3：测试单条消息到达时的未滤波话题频率计算（时间差为 2，条数为 1，频率应为 0.5）
TEST(AggregatorTest, RawFrequencyAddSingleMessage)
{
  samples::AggregatorCore aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();

  msg->timestamp = 2;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.5, aggregator.raw_frequency());
}

// 测试用例 4：测试多条消息连续到达（包含乱序时间戳）时的频率更新
TEST(AggregatorTest, RawFrequencyAddMultipleMessages)
{
  samples::AggregatorCore aggregator(0);
  auto msg = std::make_shared<sample_msgs::msg::Unfiltered>();

  msg->timestamp = 2;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.5, aggregator.raw_frequency());

  // 插入较早的时间戳，不应拉低已知的最大时间戳
  msg->timestamp = 1;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(1.0, aggregator.raw_frequency());

  // 插入更新的时间戳
  msg->timestamp = 5;
  aggregator.add_raw_msg(msg);
  EXPECT_DOUBLE_EQ(0.6, aggregator.raw_frequency());
}

// 测试用例 5：测试滤波数据包内包含无序时间戳时的处理正确性
TEST(AggregatorTest, FilteredUnorderedTimestamps)
{
  samples::AggregatorCore aggregator(0);
  auto filtered_packet = std::make_shared<sample_msgs::msg::FilteredArray>();
  std::vector<sample_msgs::msg::Filtered> msgs;
  auto msg = sample_msgs::msg::Filtered();

  msg.timestamp = 5;
  msgs.push_back(msg);
  msg.timestamp = 2;
  msgs.push_back(msg);
  msg.timestamp = 3;
  msgs.push_back(msg);
  filtered_packet->packets = msgs;

  aggregator.add_filtered_msg(filtered_packet);
  EXPECT_DOUBLE_EQ(0.2, aggregator.filtered_frequency());
}

