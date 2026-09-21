// transformer_core.cpp
// 转换器核心算法实现：包含消息解析、字符串截取提取三维坐标与环形缓冲队列

#include <string>
#include <vector>

#include "transformer_core.hpp"

namespace samples
{

/**
 * @brief 构造函数
 */
TransformerCore::TransformerCore()
{}

/**
 * @brief 返回缓冲区内部消息列表
 */
std::vector<sample_msgs::msg::Filtered> TransformerCore::buffer_messages() const
{
  return buffer_;
}

/**
 * @brief 清空内部消息缓冲区
 */
void TransformerCore::clear_buffer()
{
  buffer_.clear();
}

/**
 * @brief 校验未滤波消息有效标志位
 */
bool TransformerCore::validate_message(
  const sample_msgs::msg::Unfiltered::SharedPtr unfiltered)
{
  return unfiltered->valid;
}

/**
 * @brief 将消息加入缓存，如果未达容量上限则入队，并返回是否刚好达到容量上限
 */
bool TransformerCore::enqueue_message(const sample_msgs::msg::Filtered & msg)
{
  if (buffer_.size() < BUFFER_CAPACITY) {
    buffer_.push_back(msg);
  }
  return buffer_.size() == BUFFER_CAPACITY;
}

/**
 * @brief 解析字符串形式的三维坐标，如 "x:1.0;y:2.0;z:3.0;"
 */
bool TransformerCore::deserialize_coordinate(
  const sample_msgs::msg::Unfiltered::SharedPtr unfiltered,
  sample_msgs::msg::Filtered & filtered)
{
  std::string serialized_position = unfiltered->data;

  // 1. 查找并解析 x 坐标
  auto start_pos = serialized_position.find("x:");
  auto end_pos = serialized_position.find(";");
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
    return false;
  }
  start_pos += 2; // 偏移跳过 "x:"
  float x = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  // 2. 查找并解析 y 坐标
  start_pos = serialized_position.find("y:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
    return false;
  }
  start_pos += 2; // 偏移跳过 "y:"
  float y = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  // 3. 查找并解析 z 坐标
  start_pos = serialized_position.find("z:", end_pos + 1);
  end_pos = serialized_position.find(";", end_pos + 1);
  if (start_pos == std::string::npos || end_pos == std::string::npos ||
    end_pos < start_pos)
  {
    return false;
  }
  start_pos += 2; // 偏移跳过 "z:"
  float z = std::stof(serialized_position.substr(start_pos, end_pos - start_pos));

  // 赋值到输出结构体
  filtered.pos_x = x;
  filtered.pos_y = y;
  filtered.pos_z = z;
  return true;
}

}  // namespace samples

