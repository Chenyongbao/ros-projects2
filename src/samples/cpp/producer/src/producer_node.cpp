// producer_node.cpp
// ROS 2 生产者节点实现，声明参数并周期性发布坐标数据

#include <chrono>
#include <memory>
#include <vector>

#include "producer_node.hpp"

/**
 * @brief 构造函数：声明参数 (pos_x, pos_y, pos_z, velocity)、创建发布者与定时器
 * @param delay_ms 定时发布周期（毫秒）
 */
ProducerNode::ProducerNode(int delay_ms)
: Node("producer"), producer_(samples::ProducerCore())
{
  // 创建话题发布者
  data_pub_ =
    this->create_publisher<sample_msgs::msg::Unfiltered>("/unfiltered_topic", ADVERTISING_FREQ);

  // 创建定时器触发数据生成
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    std::bind(&ProducerNode::timer_callback, this));

  // 声明节点参数及其默认值
  this->declare_parameter("pos_x", 0.0);
  this->declare_parameter("pos_y", 0.0);
  this->declare_parameter("pos_z", 0.0);
  this->declare_parameter("velocity", 0.0);

  // 获取初始参数
  rclcpp::Parameter pos_x = this->get_parameter("pos_x");
  rclcpp::Parameter pos_y = this->get_parameter("pos_y");
  rclcpp::Parameter pos_z = this->get_parameter("pos_z");
  rclcpp::Parameter velocity = this->get_parameter("velocity");

  // 初始化核心算法对象参数
  producer_.update_position(pos_x.as_double(), pos_y.as_double(), pos_z.as_double());
  producer_.update_velocity(velocity.as_double());

  // 注册参数动态修改回调
  param_cb_ = this->add_on_set_parameters_callback(
    std::bind(&ProducerNode::parameters_callback, this, std::placeholders::_1));
}

/**
 * @brief 定时器回调函数：更新坐标，添加毫秒级时间戳并发布消息
 */
void ProducerNode::timer_callback()
{
  // 累加更新坐标
  producer_.update_coordinates();

  // 封装消息对象
  auto msg = sample_msgs::msg::Unfiltered();
  msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  producer_.serialize_coordinates(msg);

  RCLCPP_INFO(this->get_logger(), "正在发布原始数据: %s", msg.data.c_str());
  data_pub_->publish(msg);
}

/**
 * @brief 动态参数变更回调：在运行期间监听 velocity 速度参数的修改
 */
rcl_interfaces::msg::SetParametersResult ProducerNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "velocity" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      producer_.update_velocity(parameter.as_int());
      RCLCPP_INFO(this->get_logger(), "成功动态更新速度参数为: %d", parameter.as_int());
      result.successful = true;
    }
  }
  return result;
}

/**
 * @brief 节点入口主函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 启动节点，设置定时周期为 500ms
  rclcpp::spin(std::make_shared<ProducerNode>(500));
  rclcpp::shutdown();
  return 0;
}

