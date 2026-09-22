// ekf_node.cpp
// EKF 定位融合节点实现：wheel_odom 触发预测+位姿更新，IMU 触发偏航更新，
// 定时发布融合位姿 /odom/filtered —— 下游 Planner/Control/Map Memory 接口零改动

#include "ekf_node.hpp"

#include <cmath>

EkfNode::EkfNode() : Node("ekf_localizer")
{
  // Thrun 过程噪声系数与 IMU 量测噪声（参数化）
  const double a1 = declare_parameter<double>("alpha1", 0.10);
  const double a2 = declare_parameter<double>("alpha2", 0.01);
  const double a3 = declare_parameter<double>("alpha3", 0.01);
  const double a4 = declare_parameter<double>("alpha4", 0.10);
  const double sigma_imu_std = declare_parameter<double>("sigma_imu", 0.05);
  const double publish_rate = declare_parameter<double>("publish_rate", 10.0);
  sigma_imu_ = sigma_imu_std * sigma_imu_std;  // 方差
  ekf_.setMotionNoise(a1, a2, a3, a4);

  wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/wheel_odom", 10,
    std::bind(&EkfNode::wheelOdomCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", 10);

  const auto period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / std::max(1.0, publish_rate)));
  timer_ = create_wall_timer(period, std::bind(&EkfNode::publishLoop, this));

  RCLCPP_INFO(get_logger(), "ekf_localizer 就绪：/wheel_odom + /imu/data -> /odom/filtered");
}

/**
 * @brief 轮速里程计回调：噪声速度做预测，噪声位姿做观测更新
 */
void EkfNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  double dt = stamp - last_odom_stamp_sec_;
  last_odom_stamp_sec_ = stamp;
  if (dt <= 0.0 || dt > 1.0) dt = 0.1;  // 首帧或异常间隔按默认周期处理

  // 1. 预测步：控制量取轮速计的噪声速度
  ekf_.predict(msg->twist.twist.linear.x, msg->twist.twist.angular.z, dt);

  // 2. 位姿观测更新：z = 轮速计噪声位姿，R = 其协方差对角
  const double px = msg->pose.pose.position.x;
  const double py = msg->pose.pose.position.y;
  const double q = msg->pose.pose.orientation;
  const double yaw_meas = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const std::array<double, 3> z = {px, py, yaw_meas};
  const std::array<double, 3> r = {
    std::max(msg->pose.covariance[0], 1e-6),
    std::max(msg->pose.covariance[7], 1e-6),
    std::max(msg->pose.covariance[35], 1e-6)
  };
  ekf_.updatePose(z, r);
}

/**
 * @brief IMU 回调：偏航角观测更新（只修正 θ，高频纠偏）
 */
void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double q = msg->orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // 跳变保护：与上一帧差角超过 π/2 视为初始化或异常，重建基准
  if (imu_has_prev_ &&
      std::abs(std::atan2(std::sin(yaw - imu_prev_yaw_), std::cos(yaw - imu_prev_yaw_))) > M_PI / 2.0) {
    imu_has_prev_ = false;
  }
  imu_prev_yaw_ = yaw;
  imu_has_prev_ = true;

  ekf_.updateYaw(yaw, sigma_imu_);
}

/**
 * @brief 定时发布融合位姿与协方差
 */
void EkfNode::publishLoop() {
  nav_msgs::msg::Odometry out;
  out.header.stamp = now();
  out.header.frame_id = "odom";
  out.child_frame_id = "base_link";
  out.pose.pose.position.x = ekf_.x();
  out.pose.pose.position.y = ekf_.y();
  out.pose.pose.orientation.x = 0.0;
  out.pose.pose.orientation.y = 0.0;
  out.pose.pose.orientation.z = std::sin(ekf_.theta() / 2.0);
  out.pose.pose.orientation.w = std::cos(ekf_.theta() / 2.0);
  out.pose.covariance.fill(0.0);
  out.pose.covariance[0]  = ekf_.covX();
  out.pose.covariance[7]  = ekf_.covY();
  out.pose.covariance[35] = ekf_.covTheta();
  odom_pub_->publish(out);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
