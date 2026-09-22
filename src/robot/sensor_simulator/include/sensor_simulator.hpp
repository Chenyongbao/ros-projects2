#ifndef SENSOR_SIMULATOR_HPP_
#define SENSOR_SIMULATOR_HPP_

#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

/**
 * @class SensorSimulatorNode
 * @brief 传感器仿真节点：把真值速度按 Thrun 速度运动模型注入噪声，
 *        输出带漂移的轮速里程计位姿与噪声 IMU 角速度，模拟真实机器人传感器
 */
class SensorSimulatorNode : public rclcpp::Node {
  public:
    SensorSimulatorNode();

    /// 真值里程计回调：注噪并积分出噪声位姿，发布 /wheel_odom 与 /imu/data
    void odomRawCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  private:
    /// Thrun 速度运动模型：由 (v, ω) 计算线/角速度噪声标准差
    double sigmaV(double v, double omega) const;
    double sigmaW(double v, double omega) const;

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

    // 噪声模型参数（Thrun 运动模型 α1~α4）
    double alpha1_, alpha2_, alpha3_, alpha4_;
    double sigma_imu_;  ///< IMU 角速度量测噪声标准差 (rad/s)

    // 噪声积分状态
    bool initialized_ = false;
    double noisy_x_ = 0.0, noisy_y_ = 0.0, noisy_theta_ = 0.0;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

    std::mt19937 rng_;  ///< 随机数引擎

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_raw_sub_;  ///< /odom_raw 真值输入
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;   ///< /wheel_odom 噪声位姿输出
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;            ///< /imu/data 噪声角速度输出
};

#endif  // SENSOR_SIMULATOR_HPP_
