#ifndef ODOMETRY_SPOOF_NODE_HPP_
#define ODOMETRY_SPOOF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * @class OdometrySpoofNode
 * @brief 里程计伪造/转换节点类：监听仿真世界 TF 变换并数值微分推导小车线速度与角速度，生成 Odometry 消息
 */
class OdometrySpoofNode : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数，初始化发布者、TF 监听器与 10Hz 定时器
     */
    OdometrySpoofNode();

  private:
    /**
     * @brief 定时器回调函数：查询最新 TF 变换、计算差分速度并发布里程计
     */
    void timerCallback();

    // 里程计消息发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  ///< 里程计发布者 (/odom/filtered)

    // TF 变换监听工具
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                      ///< TF 坐标变换缓存
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;         ///< TF 坐标变换监听器

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;                              ///< 10Hz 定时查询定时器 (100ms)

    // 记录是否存在上一帧有效位姿
    bool has_last_transform_;                                         ///< 是否已记录前一帧 TF 数据

    // 用于微分计算速度的上一帧历史状态
    rclcpp::Time last_time_;                                          ///< 上一帧变换的时间戳
    tf2::Vector3 last_position_;                                      ///< 上一帧空间位置三维向量 (x, y, z)
    tf2::Quaternion last_orientation_;                                ///< 上一帧姿态四元数 (x, y, z, w)
};

#endif  // ODOMETRY_SPOOF_NODE_HPP_

