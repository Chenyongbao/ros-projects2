#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "control_core.hpp"

/**
 * @class ControlNode
 * @brief ROS 2 控制节点类，负责订阅路径与里程计、定时计算并发布速度控制指令
 */
class ControlNode : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数，初始化订阅者、发布者与定时器
     */
    ControlNode();

    /**
     * @brief 路径消息回调函数
     * @param msg 接收到的路径消息指针
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief 里程计消息回调函数
     * @param msg 接收到的里程计消息指针
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief 控制主循环回调函数，定时执行路径跟踪计算并发布速度指令
     */
    void controlLoop();

  private:
    robot::ControlCore control_;  ///< 控制算法核心对象

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;      ///< 路径订阅者 (/path)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< 里程计订阅者 (/odom/filtered)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;  ///< 速度控制指令发布者 (/cmd_vel)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_pub_;        ///< 卡死告警发布者 (/stuck_alert)
    rclcpp::TimerBase::SharedPtr timer_;                                 ///< 控制周期定时器 (100ms / 10Hz)
};

#endif  // CONTROL_NODE_HPP_


