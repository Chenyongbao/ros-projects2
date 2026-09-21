// transformer_test.cpp
// 转换器核心模块单元测试：包含 Google Test 测试夹具 (Fixture) 与参数化测试 (Parameterized Test)

#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "transformer_core.hpp"

/**
 * @class TransformerFixtureTest
 * @brief 测试夹具类：封装通用的测试前置准备（SetUp）与后置清理，减少重复样板代码
 */
class TransformerFixtureTest : public ::testing::Test
{
public:
  /**
   * @brief 在缓冲区中预先压入指定数量的已滤波消息
   * @param enqueue_count 预压入消息条数
   */
  void SetUp(int enqueue_count)
  {
    auto filtered = sample_msgs::msg::Filtered();
    for (int i = 0; i < enqueue_count; i++) {
      transformer.enqueue_message(filtered);
    }
  }

protected:
  samples::TransformerCore transformer;
};

/**
 * @class TransformerParameterizedTest
 * @brief 参数化测试类：使用不同输入参数（序列化字符串与期望有效性标志）多次运行相同的测试逻辑
 */
class TransformerParameterizedTest
  : public ::testing::TestWithParam<std::tuple<const char *, bool>>
{
protected:
  samples::TransformerCore transformer;
};

// 测试用例 1：校验未滤波消息如果标记为 valid=false，则验证应失败
TEST(TransformerTest, FilterInvalidField)
{
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->valid = false;

  auto transformer = samples::TransformerCore();
  bool valid = transformer.validate_message(unfiltered);
  EXPECT_FALSE(valid);
}

// 测试用例 2：测试缓冲区容量上限达到 10 条时的入队行为
TEST_F(TransformerFixtureTest, BufferCapacity)
{
  // 预先压入 9 条消息（容量 - 1）
  SetUp(samples::TransformerCore::BUFFER_CAPACITY - 1);

  // 压入第 10 条刚好填满
  auto filtered = sample_msgs::msg::Filtered();
  bool full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  int size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);

  // 缓冲区已满时再次尝试入队，依然返回满状态且大小保持为 10
  full = transformer.enqueue_message(filtered);
  EXPECT_TRUE(full);
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 10);
}

// 测试用例 3：测试清空缓冲区功能
TEST_F(TransformerFixtureTest, ClearBuffer)
{
  // 预先压入 3 条消息
  SetUp(3);

  int size = transformer.buffer_messages().size();
  EXPECT_GT(size, 0);

  // 清空缓冲区
  transformer.clear_buffer();
  size = transformer.buffer_messages().size();
  EXPECT_EQ(size, 0);
}

// 测试用例 4：参数化测试坐标反序列化的各种边界情况
TEST_P(TransformerParameterizedTest, SerializationValidation)
{
  auto [serialized_field, valid] = GetParam();
  auto filtered = sample_msgs::msg::Filtered();
  auto unfiltered = std::make_shared<sample_msgs::msg::Unfiltered>();
  unfiltered->data = serialized_field;
  EXPECT_EQ(transformer.deserialize_coordinate(unfiltered, filtered), valid);
}

// 实例化参数化测试：覆盖缺少分号、坐标乱序、非法分隔符、缺少轴以及合法格式等多种边界测试用例
INSTANTIATE_TEST_CASE_P(
  Serialization, TransformerParameterizedTest,
  ::testing::Values(
    std::make_tuple("x:1;y:2;z:3", false),      // 错误：缺少末尾分号
    std::make_tuple("z:1;y:2;x:3;", false),     // 错误：顺序颠倒
    std::make_tuple("x:1,y:2,z:3", false),      // 错误：使用了逗号而不是分号
    std::make_tuple("x:3;", false),             // 错误：缺少 y 和 z
    std::make_tuple("x:3;y:2;z:3;", true),      // 正确：标准格式
    std::make_tuple("x:3;y:22; z:11;", true))); // 正确：包含空格的合法格式

