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
 * @struct ProgressMonitor
 * @brief 卡死检测监控器状态：时间窗口起点快照 + 当前卡死标志
 */
struct ProgressMonitor {
  bool initialized = false;        ///< 是否已记录窗口起点
  double window_start_x = 0.0;     ///< 窗口起点 X（米）
  double window_start_y = 0.0;     ///< 窗口起点 Y（米）
  double window_start_time = 0.0;  ///< 窗口起点时间（秒）
  double window_start_goal_dist = 0.0; ///< 窗口起点时到目标的距离（米）
  bool is_stuck = false;           ///< 当前是否判定卡死
};

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

    /**
     * @brief 更新卡死检测监控器（每个控制周期调用）
     * @param now 当前时间（秒）
     * @param cmd 本周期下发的速度指令（用于排除正常停车）
     * @return true 本周期判定为卡死
     */
    bool updateStuckDetection(double now, const geometry_msgs::msg::Twist& cmd);

    /**
     * @brief 当前是否处于卡死状态
     */
    bool isStuck() const { return monitor_.is_stuck; }

    /**
     * @brief 通知已执行脱困动作，重置监控窗口（避免误报连续卡死）
     */
    void resetStuckMonitor() { monitor_.initialized = false; monitor_.is_stuck = false; }

    /**
     * @brief 是否正处于脱困摆动阶段（节点层据此反转指令）
     */
    bool inRecovery() const { return recovery_until_ > 0.0; }

    /**
     * @brief 请求进入脱困摆动阶段
     * @param now 当前时间（秒） @param duration 摆动持续时长（秒）
     */
    void enterRecovery(double now, double duration) { recovery_until_ = now + duration; }

    /**
     * @brief 计算脱困摆动指令（线速度/角速度反转）
     */
    geometry_msgs::msg::Twist computeRecoveryCommand(double now);

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

    // 卡死检测参数与状态
    ProgressMonitor monitor_;              ///< 进度监控器
    double stuck_window_ = 3.0;            ///< 检测时间窗（秒）
    double min_progress_dist_ = 0.05;      ///< 窗口内最小位移阈值（米）
    double min_goal_improvement_ = 0.05;   ///< 窗口内到目标距离最小改善量（米）
    double recovery_until_ = 0.0;          ///< 脱困摆动截止时间（秒，0 表示不在脱困中）

    /**
     * @brief 在路径中寻找合适的前瞻目标点
     */
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;

    /**
     * @brief 计算机器人在路径上的投影弧长进度（当前位姿到路径最近投影点的累计弧长）
     */
    double computePathProgress() const;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_

