#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

/**
 * @struct CellIndex
 * @brief 栅格地图二维索引坐标结构体 (x, y)
 */
struct CellIndex {
  int x;  ///< 栅格在 X 轴方向的列索引
  int y;  ///< 栅格在 Y 轴方向的行索引
  
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

/**
 * @struct CellIndexHash
 * @brief 自定义哈希函数对象，支持将 CellIndex 作为 std::unordered_map 的键
 */
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

/**
 * @struct AStarNode
 * @brief A* 优先队列中的搜索节点，包含栅格坐标及其对应的总代价评估值 f_score (f = g + h)
 */
struct AStarNode {
  CellIndex index;   ///< 节点对应的栅格坐标
  double f_score;    ///< 综合代价评估值：当前起点到该点的已知代价 g 与到终点的预估代价 h 之和
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

/**
 * @struct CompareF
 * @brief 优先队列比较仿函数，使得优先队列表现为小顶堆（优先弹出 f_score 最小的节点）
 */
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    // std::priority_queue 默认为大顶堆，因此当 a.f_score > b.f_score 时返回 true 构建小顶堆
    return a.f_score > b.f_score;
  }
};

/**
 * @class PlannerCore
 * @brief 路径规划核心算法实现类（基于 8 邻域 A* 搜索算法）
 */
class PlannerCore {
  public:
    /**
     * @brief 构造函数
     * @param logger ROS 2 日志记录器
     */
    explicit PlannerCore(const rclcpp::Logger& logger);

    /**
     * @brief 更新当前全局/局部占据栅格地图
     */
    void updateMap(const nav_msgs::msg::OccupancyGrid& msg);

    /**
     * @brief 更新目标点位置
     */
    void updateGoal(const geometry_msgs::msg::PointStamped& msg);

    /**
     * @brief 更新机器人当前在世界坐标系下的坐标
     */
    void updateOdometry(double x, double y);

    /**
     * @brief 执行 A* 算法搜索最优路径
     * @return nav_msgs::msg::Path 规划出的无碰撞路径点序列
     */
    nav_msgs::msg::Path planPath();

    /**
     * @brief 判断当前是否需要重新规划路径
     */
    bool shouldReplan() const;

    /**
     * @brief 判断机器人是否已到达目标点
     */
    bool goalReached() const;

    /**
     * @brief 标记已到达目标，重置规划器状态为等待新目标
     */
    void markGoalReached();

  private:
    rclcpp::Logger logger_;  ///< ROS 2 日志记录器

    /// 规划器状态枚举：等待目标点 / 正在前往目标点
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_ = State::WAITING_FOR_GOAL;  ///< 当前规划器运行状态

    nav_msgs::msg::OccupancyGrid map_;      ///< 最新的占据栅格地图
    double goal_x_ = 0.0;                   ///< 目标点在世界坐标系下的 X 坐标（米）
    double goal_y_ = 0.0;                   ///< 目标点在世界坐标系下的 Y 坐标（米）
    double robot_x_ = 0.0;                  ///< 机器人当前在世界坐标系下的 X 坐标（米）
    double robot_y_ = 0.0;                  ///< 机器人当前在世界坐标系下的 Y 坐标（米）

    bool has_map_ = false;                  ///< 是否已接收到地图
    bool has_goal_ = false;                 ///< 是否已设置有效目标

    /**
     * @brief 世界坐标系 (米) 转换为栅格坐标系 (网格索引)
     * @param wx 世界坐标 X
     * @param wy 世界坐标 Y
     * @param cx 输出栅格列坐标
     * @param cy 输出栅格行坐标
     * @return true 坐标在地图范围内；false 超出地图边界
     */
    bool worldToGrid(double wx, double wy, int& cx, int& cy) const;

    /**
     * @brief 栅格坐标系 (网格索引) 转换为世界坐标系 (米)，定位在栅格中心
     */
    void gridToWorld(int cx, int cy, double& wx, double& wy) const;

    /**
     * @brief 检查栅格单元是否被障碍物占据
     * @param cx 栅格列索引
     * @param cy 栅格行索引
     * @return true 障碍物或不可通行；false 自由区域可通行
     */
    bool isOccupied(int cx, int cy) const;

    /**
     * @brief A* 启发式函数（计算两栅格间的欧几里得距离）
     */
    double heuristic(const CellIndex& a, const CellIndex& b) const;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_

