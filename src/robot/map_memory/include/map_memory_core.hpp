#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

namespace robot
{

/**
 * @class MapMemoryCore
 * @brief 地图记忆与融合核心类，负责将局部代价地图通过小车位姿仿射变换持续融合至全局占据栅格地图中
 */
class MapMemoryCore {
  public:
    /**
     * @brief 构造函数，初始化全局地图大小与默认未知状态 (-1)
     * @param logger ROS 2 日志记录器
     */
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    /**
     * @brief 接收并缓存最新的局部代价地图
     */
    void updateCostmap(const nav_msgs::msg::OccupancyGrid& msg);

    /**
     * @brief 接收里程计更新，提取小车当前位置和航向角
     */
    void updateOdometry(const nav_msgs::msg::Odometry& msg);

    /**
     * @brief 尝试触发地图融合（当小车累计位移超过设定阈值时执行）
     * @return true 成功执行融合；false 移动距离未达阈值或尚无代价地图
     */
    bool tryMerge();

    /**
     * @brief 获取融合生成的全局占据栅格地图消息
     * @return nav_msgs::msg::OccupancyGrid
     */
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    
  private:
    rclcpp::Logger logger_;  ///< ROS 2 日志记录器

    // 全局地图几何参数
    double resolution_ = 0.2;   ///< 全局地图分辨率（米/栅格），每个单元格为 0.2m x 0.2m
    int width_ = 150;           ///< 全局地图宽度（栅格数），对应 30 米
    int height_ = 150;          ///< 全局地图高度（栅格数），对应 30 米
    double origin_x_ = -15.0;   ///< 全局地图原点 X 坐标（米）
    double origin_y_ = -15.0;   ///< 全局地图原点 Y 坐标（米）
    std::vector<int8_t> global_map_;  ///< 全局占据栅格数据一维数组（-1: 未知, 0: 自由, 1-100: 障碍物概率）

    // 接收到的最新局部数据
    nav_msgs::msg::OccupancyGrid latest_costmap_;  ///< 最新的局部代价地图缓存
    bool has_costmap_ = false;                     ///< 是否已接收到局部代价地图

    double robot_x_ = 0.0;      ///< 机器人当前 X 坐标（米）
    double robot_y_ = 0.0;      ///< 机器人当前 Y 坐标（米）
    double robot_yaw_ = 0.0;    ///< 机器人当前偏航角（弧度）

    // 上一次执行融合时的小车位姿（用于位移距离阈值判定）
    double last_x_ = 0.0;       ///< 上次融合时小车 X 坐标
    double last_y_ = 0.0;       ///< 上次融合时小车 Y 坐标
    bool has_last_ = false;     ///< 是否已记录过历史位姿

    double distance_threshold_ = 0.3;  ///< 融合触发距离阈值（米）：移动超过 0.3m 才触发新一轮融合，降低计算开销

    /**
     * @brief 将当前局部代价地图坐标经旋转平移后栅格化写入全局地图
     */
    void mergeLatestCostmap();
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_

