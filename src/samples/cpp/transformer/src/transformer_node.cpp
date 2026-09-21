// transformer_node.cpp
// ROS 2 转换器节点实现：接收未滤波数据，解析三维坐标并批量封装发布

#include <memory>

#include "transformer_node.hpp"

/**
 * @brief 构造函数：初始化订阅者、发布者与元数据参数 (version, compression_method)
 */
TransformerNode::TransformerNode()
: Node("transformer"), transformer_(samples::TransformerCore())
{
  // 订阅原始未滤波数据话题
  raw_sub_ = this->create_subscription<sample_msgs::msg::Unfiltered>(
    "/unfiltered_topic", ADVERTISING_FREQ,
    std::bind(
      &TransformerNode::unfiltered_callback, this,
      std::placeholders::_1));

  // 创建批量滤波数据发布者
  transform_pub_ =
    this->create_publisher<sample_msgs::msg::FilteredArray>("/filtered_topic", ADVERTISING_FREQ);

  // 声明节点参数（支持通过 params.yaml 加载）
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

/**
 * @brief 原始数据消息订阅回调函数
 * 1. 校验消息合法性；2. 反序列化坐标；3. 附加元数据；4. 压入缓冲区，满 10 条则打包发布
 */
void TransformerNode::unfiltered_callback(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  // 1. 校验消息是否有效
  if (!transformer_.validate_message(msg)) {
    return;
  }

  // 2. 反序列化解析三维坐标
  auto filtered = sample_msgs::msg::Filtered();
  if (!transformer_.deserialize_coordinate(msg, filtered)) {
    return;
  }

  // 3. 补充时间戳与参数元数据
  filtered.timestamp = msg->timestamp;
  filtered.metadata.version = this->get_parameter("version").as_int();
  filtered.metadata.compression_method =
    this->get_parameter("compression_method").as_int();

  // 4. 将处理后的单条数据存入缓冲区队列
  if (transformer_.enqueue_message(filtered)) {
    RCLCPP_INFO(this->get_logger(), "缓冲区达到最大容量，正在批量发布处理结果 (PUBLISHING)...");
    
    // 构建批量发布数组消息
    sample_msgs::msg::FilteredArray filtered_msgs;
    auto buffer = transformer_.buffer_messages();
    transformer_.clear_buffer(); // 清空缓冲区以接收下一批数据

    for (auto & packet : buffer) {
      filtered_msgs.packets.push_back(packet);
    }
    // 发布到 /filtered_topic
    transform_pub_->publish(filtered_msgs);
  }
}

/**
 * @brief 节点入口主函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformerNode>());
  rclcpp::shutdown();
  return 0;
}

