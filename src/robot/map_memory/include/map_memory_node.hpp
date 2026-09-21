#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

/**
 * @class MapMemoryNode
 * @brief ROS 2 地图记忆节点类，订阅局部代价地图与里程计，周期性融合并发布全局占据栅格地图 /map
 */
class MapMemoryNode : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数，初始化订阅者、发布者与融合发布定时器
     */
    MapMemoryNode();

    /**
     * @brief 局部代价地图接收回调
     * @param msg 局部代价地图共享指针
     */
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 滤波后里程计接收回调
     * @param msg 里程计消息共享指针
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief 定时器主回调：尝试地图融合并将全局地图发布至 /map 话题
     */
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;  ///< 地图记忆核心算法对象

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;  ///< 局部代价地图订阅者 (/costmap)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;          ///< 滤波后里程计订阅者 (/odom/filtered)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;         ///< 全局占据栅格地图发布者 (/map)
    rclcpp::TimerBase::SharedPtr timer_;                                         ///< 定时更新与发布定时器 (1秒 / 1Hz)
};

#endif  // MAP_MEMORY_NODE_HPP_

