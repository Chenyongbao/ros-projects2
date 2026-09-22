#ifndef EKF_NODE_HPP_
#define EKF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "ekf_core.hpp"

/**
 * @class EkfNode
 * @brief EKF 定位融合节点：订阅带噪轮速里程计与 IMU，输出融合位姿 /odom/filtered
 */
class EkfNode : public rclcpp::Node {
  public:
    EkfNode();

    /// 轮速里程计回调：预测步 + 位姿观测更新
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// IMU 回调：偏航角观测更新
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  private:
    /// 定时发布融合后的位姿（10Hz）
    void publishLoop();

    robot::EkfCore ekf_;  ///< 滤波器核心

    bool imu_has_prev_ = false;   ///< IMU 是否已有上一帧（用于跳变保护）
    double imu_prev_yaw_ = 0.0;   ///< IMU 上一帧 yaw

    double sigma_imu_ = 0.05;     ///< IMU yaw 观测噪声方差（参数换算）
    double last_odom_stamp_sec_ = 0.0;  ///< 上一帧轮速里程计时间戳

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;  ///< /wheel_odom
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;           ///< /imu/data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;           ///< /odom/filtered
    rclcpp::TimerBase::SharedPtr timer_;                                       ///< 发布定时器
};

#endif  // EKF_NODE_HPP_
