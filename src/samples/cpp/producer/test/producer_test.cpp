// producer_test.cpp
// 生产者核心功能单元测试，使用 Google Test 框架验证序列化与坐标累加逻辑

#include "gtest/gtest.h"

#include "producer_core.hpp"

// 测试用例 1：测试合法非零坐标的字符串序列化正确性
TEST(ProducerTest, ValidSerialization)
{
  auto producer = samples::ProducerCore(11, -12, 0);
  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);

  EXPECT_TRUE(msg.valid);
  EXPECT_EQ(msg.data, "x:11.000000;y:-12.000000;z:0.000000;");
}

// 测试用例 2：测试当速度为 0 时更新坐标，坐标值应保持不变
TEST(ProducerTest, NoCoordinateUpdate)
{
  auto producer = samples::ProducerCore();
  producer.update_coordinates();

  auto msg = sample_msgs::msg::Unfiltered();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:0.000000;y:0.000000;z:0.000000;");
}

// 测试用例 3：测试多次设置不同速度（正向与负向）时的坐标累加计算正确性
TEST(ProducerTest, MultipleCoordinateUpdate)
{
  auto producer = samples::ProducerCore();
  auto msg = sample_msgs::msg::Unfiltered();

  // 设置正速度 1
  producer.update_velocity(1);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:0.577350;y:0.577350;z:0.577350;");

  // 设置负速度 -4
  producer.update_velocity(-4);
  producer.update_coordinates();
  producer.serialize_coordinates(msg);
  EXPECT_EQ(msg.data, "x:-1.732051;y:-1.732051;z:-1.732051;");
}

