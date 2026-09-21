#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

/**
 * @class ControlCore
 * @brief 机器人路径跟踪核心算法类（基于纯追踪/前瞻点跟踪算法）
 */
class ControlCore {
  public:
    /**
     * @brief 构造函数，传入 ROS 2 节点的 Logger 实例以支持终端日志输出
     */
    ControlCore(const rclcpp::Logger& logger);

    /**
     * @brief 更新当前跟踪的路径数据
     */
    void updatePath(const nav_msgs::msg::Path& msg);

    /**
     * @brief 更新机器人当前里程计位姿
     */
    void updateOdometry(const nav_msgs::msg::Odometry& msg);

    /**
     * @brief 计算并生成小车的速度控制指令（cmd_vel）
     */
    geometry_msgs::msg::Twist computeCommand();

    /**
     * @brief 检查当前是否有可跟踪的有效路径
     */
    bool hasPath() const;

  private:
    rclcpp::Logger logger_;  ///< ROS 2 日志记录器

    nav_msgs::msg::Path path_;  ///< 当前跟踪的目标路径
    bool has_path_ = false;     ///< 是否已接收到有效路径

    double robot_x_ = 0.0;      ///< 机器人当前 X 坐标（米）
    double robot_y_ = 0.0;      ///< 机器人当前 Y 坐标（米）
    double robot_yaw_ = 0.0;    ///< 机器人当前航向角（偏航角 Yaw，弧度）

    // 控制器参数
    double lookahead_dist_ = 0.5;      ///< 前瞻距离（米）：向前搜索目标点的距离
    double linear_speed_ = 1.0;        ///< 前进巡航线速度（米/秒）
    double goal_tolerance_ = 0.3;      ///< 目标点容差（米）：距离路径终点小于该值时停止
    double max_steering_angle_ = 0.5;  ///< 最大转向角阈值（弧度）：航向误差大于此值时原地旋转

    /**
     * @brief 在路径中寻找合适的前瞻目标点
     */
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_

