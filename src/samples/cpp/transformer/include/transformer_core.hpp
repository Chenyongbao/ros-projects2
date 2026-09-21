#ifndef TRANSFORMER_CORE_HPP_
#define TRANSFORMER_CORE_HPP_

#include <vector>

#include "sample_msgs/msg/unfiltered.hpp"
#include "sample_msgs/msg/filtered.hpp"

namespace samples
{

/**
 * @class TransformerCore
 * @brief 转换器核心处理逻辑类，负责数据有效性检验、坐标反序列化以及批量缓存队列管理
 */
class TransformerCore
{
public:
  // 缓冲区容量：达到 10 条处理后的消息后触发批量发布
  static constexpr int BUFFER_CAPACITY = 10;

public:
  /**
   * @brief 构造函数
   */
  TransformerCore();

  /**
   * @brief 获取当前缓冲区中排队的所有已滤波消息
   * @return std::vector<sample_msgs::msg::Filtered> 消息列表
   */
  std::vector<sample_msgs::msg::Filtered> buffer_messages() const;

  /**
   * @brief 清空当前消息缓冲区（在消息批量发布后调用）
   */
  void clear_buffer();

  /**
   * @brief 校验未滤波原始消息的 valid 标志位是否为 true
   * @param unfiltered 原始输入消息
   * @return true 消息合法有效；false 消息无效
   */
  bool validate_message(
    const sample_msgs::msg::Unfiltered::SharedPtr unfiltered);

  /**
   * @brief 将已解析的消息压入缓冲区队列，当达到 BUFFER_CAPACITY 时忽略多余消息
   * @param msg 已解析的结构化消息
   * @return true 压入后缓冲区已满；false 缓冲区尚未填满
   */
  bool enqueue_message(const sample_msgs::msg::Filtered & msg);

  /**
   * @brief 反序列化原始消息中的 data 字符串字段（格式为 "x:$num1;y:$num2;z:$num3;"）
   * @param[in] unfiltered 包含序列化字符串的原始输入消息
   * @param[out] filtered 解析后填充的三维坐标结构化消息
   * @return true 解析成功；false 字符串格式不匹配或解析失败
   */
  bool deserialize_coordinate(
    const sample_msgs::msg::Unfiltered::SharedPtr unfiltered,
    sample_msgs::msg::Filtered & filtered);

private:
  // 消息暂存缓冲区，容量达到 BUFFER_CAPACITY 后清空
  std::vector<sample_msgs::msg::Filtered> buffer_;
};

}  // namespace samples

#endif  // TRANSFORMER_CORE_HPP_

