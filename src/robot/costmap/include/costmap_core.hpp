#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot
{

/**
 * @class CostmapCore
 * @brief 局部代价地图（Costmap）核心算法实现类，负责将激光雷达点云转换为占据栅格地图并执行障碍物膨胀
 */
class CostmapCore {
  public:
    /**
     * @brief 构造函数，初始化日志记录器与栅格容器
     * @param logger ROS 2 日志记录器
     */
    explicit CostmapCore(const rclcpp::Logger& logger);

    /**
     * @brief 初始化/重置栅格地图，将所有单元格代价清零
     */
    void initializeCostmap();

    /**
     * @brief 将激光雷达极坐标测距点 (range, angle) 投影并转换为栅格网格索引 (x_cell, y_cell)
     * @param range 测量距离（米）
     * @param angle 测量方位角（弧度）
     * @param x_cell 输出栅格列索引
     * @param y_cell 输出栅格行索引
     * @return true 转换成功且坐标落在局部地图范围内；false 越界
     */
    bool convertToGrid(double range, double angle, int& x_cell, int& y_cell) const;

    /**
     * @brief 将指定栅格标记为确定的障碍物（赋予最大代价值 max_cost_）
     * @param x_cell 栅格列索引
     * @param y_cell 栅格行索引
     */
    void markObstacle(int x_cell, int y_cell);

    /**
     * @brief 根据最新接收到的激光雷达扫描数据更新代价地图
     * @param scan 激光雷达扫描消息
     */
    void updateFromScan(const sensor_msgs::msg::LaserScan& scan);

    /**
     * @brief 获取生成的 ROS 标准占据栅格地图消息对象
     * @return nav_msgs::msg::OccupancyGrid
     */
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;

  private:
    rclcpp::Logger logger_;  ///< ROS 2 日志记录器
    
    // 代价地图核心几何与尺寸配置
    double resolution_ = 0.1;       ///< 地图分辨率（米/栅格），每个单元格代表 0.1m x 0.1m
    int width_ = 200;               ///< 地图宽度（栅格数），对应 20 米
    int height_ = 200;              ///< 地图高度（栅格数），对应 20 米
    double origin_x_ = -10.0;       ///< 地图左下角原点在局部坐标系中的 X 偏移（米），使小车居中
    double origin_y_ = -10.0;       ///< 地图左下角原点在局部坐标系中的 Y 偏移（米），使小车居中
    int max_cost_ = 100;            ///< 障碍物最大代价值 (0-100)
    double inflation_radius_ = 1.0; ///< 障碍物膨胀安全半径（米）
    std::vector<int8_t> grid_;      ///< 一维连续存储的二维网格代价值数据数组

    /**
     * @brief 对已标记的硬障碍物执行安全缓冲膨胀算法（距离越近代价越高）
     */
    void inflateObstacles();
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_

