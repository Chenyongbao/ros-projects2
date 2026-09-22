// sensor_simulator.cpp
// 传感器仿真节点实现：按 Thrun 速度运动模型给真值速度注入高斯噪声，
// 欧拉积分出漂移位姿发布 /wheel_odom（含协方差），同时发布噪声 IMU 角速度 /imu/data
//
// 参考 taorobot odometry_noise_node，并扩展 IMU 通道：
//   σ²_v = α1·v² + α2·ω²   （线速度噪声）
//   σ²_ω = α3·v² + α4·ω²   （角速度噪声）

#include "sensor_simulator.hpp"

#include <cmath>

SensorSimulatorNode::SensorSimulatorNode() : Node("sensor_simulator"),
  alpha1_(declare_parameter<double>("alpha1", 0.10)),
  alpha2_(declare_parameter<double>("alpha2", 0.01)),
  alpha3_(declare_parameter<double>("alpha3", 0.01)),
  alpha4_(declare_parameter<double>("alpha4", 0.10)),
  sigma_imu_(declare_parameter<double>("sigma_imu", 0.05)),
  rng_(std::random_device{}())
{
  // 真值输入（odometry_spoof 改发到此话题）
  odom_raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom_raw", 10,
    std::bind(&SensorSimulatorNode::odomRawCallback, this, std::placeholders::_1));

  // 带噪传感器输出
  wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

  RCLCPP_INFO(get_logger(),
    "sensor_simulator 就绪：/odom_raw -> /wheel_odom + /imu/data | α1=%.3f α2=%.3f α3=%.3f α4=%.3f σ_imu=%.3f",
    alpha1_, alpha2_, alpha3_, alpha4_, sigma_imu_);
}

/**
 * @brief 由 (v, ω) 计算 Thrun 速度运动模型的线速度噪声标准差
 */
double SensorSimulatorNode::sigmaV(double v, double omega) const {
  return std::sqrt(alpha1_ * v * v + alpha2_ * omega * omega);
}

/**
 * @brief 由 (v, ω) 计算 Thrun 速度运动模型的角速度噪声标准差
 */
double SensorSimulatorNode::sigmaW(double v, double omega) const {
  return std::sqrt(alpha3_ * v * v + alpha4_ * omega * omega);
}

/**
 * @brief 真值里程计回调：注噪积分 → 发布噪声轮速里程计与 IMU 角速度
 */
void SensorSimulatorNode::odomRawCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const rclcpp::Time stamp(msg->header.stamp);

  // 首帧：初始化噪声位姿为真值起点
  if (!initialized_) {
    noisy_x_ = msg->pose.pose.position.x;
    noisy_y_ = msg->pose.pose.position.y;
    noisy_theta_ = yawFromQuaternion(msg->pose.pose.orientation);
    last_stamp_ = stamp;
    initialized_ = true;
    return;
  }

  const double dt = (stamp - last_stamp_).seconds();
  last_stamp_ = stamp;
  if (dt <= 0.0 || dt > 1.0) return;

  // 真值速度作为运动控制量
  const double v = msg->twist.twist.linear.x;
  const double omega = msg->twist.twist.angular.z;

  // 按运动模型采样噪声速度
  const double v_noisy = v + std::normal_distribution<double>(0.0, sigmaV(v, omega))(rng_);
  const double w_noisy = omega + std::normal_distribution<double>(0.0, sigmaW(v, omega))(rng_);

  // 欧拉积分噪声速度得到漂移位姿
  noisy_x_ += v_noisy * std::cos(noisy_theta_) * dt;
  noisy_y_ += v_noisy * std::sin(noisy_theta_) * dt;
  noisy_theta_ += w_noisy * dt;
  noisy_theta_ = std::atan2(std::sin(noisy_theta_), std::cos(noisy_theta_));  // 归一化 [-π, π]

  const auto q = quaternionFromYaw(noisy_theta_);

  // --- 发布噪声轮速里程计（位姿 + 对角协方差） ---
  nav_msgs::msg::Odometry out;
  out.header = msg->header;
  out.header.frame_id = "odom";
  out.child_frame_id = "base_link";
  out.pose.pose.position.x = noisy_x_;
  out.pose.pose.position.y = noisy_y_;
  out.pose.pose.orientation = q;

  const double cov_pos = sigmaV(v, omega) * dt * sigmaV(v, omega) * dt;
  const double cov_yaw = sigmaW(v, omega) * dt * sigmaW(v, omega) * dt;
  out.pose.covariance.fill(0.0);
  out.pose.covariance[0]  = cov_pos;  // x
  out.pose.covariance[7]  = cov_pos;  // y
  out.pose.covariance[35] = cov_yaw;  // yaw
  out.twist.twist.linear.x = v_noisy;
  out.twist.twist.angular.z = w_noisy;
  wheel_odom_pub_->publish(out);

  // --- 发布噪声 IMU 角速度（独立采样，模拟陀螺仪） ---
  sensor_msgs::msg::Imu imu;
  imu.header = msg->header;
  imu.header.frame_id = "base_link";
  imu.angular_velocity.z = w_noisy + std::normal_distribution<double>(0.0, sigma_imu_)(rng_);
  imu.angular_velocity_covariance.fill(0.0);
  imu.angular_velocity_covariance[8] = sigma_imu_ * sigma_imu_;  // 绕 Z 轴方差
  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = std::sin(noisy_theta_ / 2.0);
  imu.orientation.w = std::cos(noisy_theta_ / 2.0);
  imu_pub_->publish(imu);
}

double SensorSimulatorNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion SensorSimulatorNode::quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
