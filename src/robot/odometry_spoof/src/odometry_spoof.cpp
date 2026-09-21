// odometry_spoof.cpp
// 里程计欺骗节点实现：监听仿真环境中小车的世界坐标 TF，计算微分线速度与角速度并封装发布里程计数据

#include <chrono>
#include <memory>

#include "odometry_spoof.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @brief 构造函数
 * 初始化里程计发布者 (odom/filtered)、TF 变换监听缓存与 10Hz 定时查询器
 */
OdometrySpoofNode::OdometrySpoofNode() : Node("odometry_spoof") {
  // 创建里程计消息发布者
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 10);

  // 创建 TF2 缓存与监听器实例
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建周期性定时器：每隔 100ms（频率 10Hz）执行一次查询与发布
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&OdometrySpoofNode::timerCallback, this)
  );
}

/**
 * @brief 定时器回调函数：查询从世界坐标系到小车雷达基准坐标系的坐标变换，推导速度并发布 Odometry
 */
OdometrySpoofNode::timerCallback() {
  // 定义源坐标系与目标坐标系
  // 说明：在仿真中以 lidar 作为底盘位姿参考，便于作业处理
  const std::string target_frame = "robot/chassis/lidar";
  const std::string source_frame = "sim_world";

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    // 查询当前可用的最新 TF 变换 (TimePointZero)
    transform_stamped = tf_buffer_->lookupTransform(
      source_frame,
      target_frame,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException &ex) {
    // 查询失败时限流打印警告信息
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无法将坐标系 %s 转换至 %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return;
  }

  // 构建待发布的 Odometry 消息
  nav_msgs::msg::Odometry odom_msg;

  // 填写头部信息
  odom_msg.header.stamp = transform_stamped.header.stamp;
  odom_msg.header.frame_id = source_frame;  // 参考世界坐标系 "sim_world"
  odom_msg.child_frame_id  = target_frame;  // 机器人子坐标系

  // 从 TF 变换中提取机器人的空间位置和姿态四元数
  odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
  odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
  odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
  odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

  // --- 根据相邻两次 TF 变换的差值与时间间隔计算速度 Twist ---
  if (has_last_transform_)
  {
    // 计算两帧之间的时间差 dt（秒）
    rclcpp::Time current_time = transform_stamped.header.stamp;
    double dt = (current_time - last_time_).seconds();

    if (dt > 0.0)
    {
      // 1. 计算三维线速度 (dx/dt, dy/dt, dz/dt)
      double dx = odom_msg.pose.pose.position.x - last_position_.x();
      double dy = odom_msg.pose.pose.position.y - last_position_.y();
      double dz = odom_msg.pose.pose.position.z - last_position_.z();

      odom_msg.twist.twist.linear.x = dx / dt;
      odom_msg.twist.twist.linear.y = dy / dt;
      odom_msg.twist.twist.linear.z = dz / dt;

      // 2. 计算三维角速度 (通过前后两帧四元数相对旋转推导)
      tf2::Quaternion q_last(last_orientation_.x(),
                             last_orientation_.y(),
                             last_orientation_.z(),
                             last_orientation_.w());

      tf2::Quaternion q_current(transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y,
                                transform_stamped.transform.rotation.z,
                                transform_stamped.transform.rotation.w);

      // 姿态旋转差值：q_diff = q_last^-1 * q_current
      tf2::Quaternion q_diff = q_last.inverse() * q_current;

      // 从相对四元数中提取滚转角(roll)、俯仰角(pitch)与偏航角(yaw)增量
      double roll_diff, pitch_diff, yaw_diff;
      tf2::Matrix3x3(q_diff).getRPY(roll_diff, pitch_diff, yaw_diff);

      // 计算角速度（弧度/秒）
      odom_msg.twist.twist.angular.x = roll_diff  / dt;
      odom_msg.twist.twist.angular.y = pitch_diff / dt;
      odom_msg.twist.twist.angular.z = yaw_diff   / dt;
    }
    else
    {
      // 若时间间隔 dt <= 0，则速度归零
      odom_msg.twist.twist.linear.x  = 0.0;
      odom_msg.twist.twist.linear.y  = 0.0;
      odom_msg.twist.twist.linear.z  = 0.0;
      odom_msg.twist.twist.angular.x = 0.0;
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
    }
  }
  else
  {
    // 首次接收变换尚无上一帧参考，将速度初始化为 0
    odom_msg.twist.twist.linear.x  = 0.0;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // 标记已获取到初始帧
    has_last_transform_ = true;
  }

  // 发布里程计消息
  odom_pub_->publish(odom_msg);

  // 保存当前位姿与时间戳，作为下一次循环的历史参考基准
  last_time_ = transform_stamped.header.stamp;
  last_position_.setValue(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z
  );
  last_orientation_.setValue(
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w
  );
}

/**
 * @brief 节点主入口函数
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySpoofNode>());
  rclcpp::shutdown();
  return 0;
}

