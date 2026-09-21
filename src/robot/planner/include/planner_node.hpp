#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

/**
 * @class PlannerNode
 * @brief ROS 2 路径规划节点类，订阅地图、目标点与里程计，定时或事件触发路径规划并发布结果
 */
class PlannerNode : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数，初始化话题订阅者、发布者与定时器
     */
    PlannerNode();

    /**
     * @brief 占据栅格地图消息回调函数
     * @param msg 接收到的地图消息指针
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 目标点消息回调函数
     * @param msg 接收到的目标点消息指针
     */
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief 里程计消息回调函数
     * @param msg 接收到的里程计消息指针
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief 定时器回调函数，周期性检测小车是否已到达目标点
     */
    void timerCallback();

  private:
    robot::PlannerCore planner_;  ///< 路径规划算法核心对象

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;              ///< 地图订阅者 (/map)
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;   ///< 目标点订阅者 (/goal_point)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                  ///< 里程计订阅者 (/odom/filtered)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;                         ///< 规划路径发布者 (/path)
    rclcpp::TimerBase::SharedPtr timer_;                                                 ///< 状态检测定时器 (500ms / 2Hz)
};

#endif  // PLANNER_NODE_HPP_

