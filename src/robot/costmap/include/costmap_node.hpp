#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

/**
 * @class CostmapNode
 * @brief ROS 2 代价地图节点，订阅激光雷达扫描话题并实时发布膨胀后的局部占据代价地图
 */
class CostmapNode : public rclcpp::Node {
  public:
    /**
     * @brief 构造函数，初始化订阅者与发布者
     */
    CostmapNode();

    /**
     * @brief 激光雷达扫描消息回调函数
     * @param scan 激光雷达扫描消息共享指针
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;  ///< 代价地图核心算法对象

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;  ///< 激光雷达订阅者 (/lidar)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;   ///< 局部代价地图发布者 (/costmap)
};

#endif  // COSTMAP_NODE_HPP_

