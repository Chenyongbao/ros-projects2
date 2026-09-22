#ifndef MISSION_MANAGER_CORE_HPP_
#define MISSION_MANAGER_CORE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

/**
 * @brief 任务管理状态机状态
 */
enum class MissionState {
  IDLE,    ///< 空闲：无任务或任务全部完成
  ACTIVE,  ///< 执行中：已下发当前目标，等待到达反馈
  RETRY    ///< 重试中：收到卡死/失败告警，准备重新下发当前目标
};

/**
 * @struct MissionStatus
 * @brief 对外发布的状态快照（由节点层序列化为 JSON 字符串）
 */
struct MissionStatus {
  MissionState state = MissionState::IDLE;
  int current_index = 0;   ///< 当前执行的任务点序号（1 起，0 表示无）
  int total = 0;           ///< 队列中任务点总数
  int retry_count = 0;     ///< 当前任务点的重试次数
  int finished = 0;        ///< 已完成（到达）的任务点数
  int skipped = 0;         ///< 已跳过（重试耗尽）的任务点数
};

/**
 * @class MissionManagerCore
 * @brief 任务管理核心：多点任务队列 + 状态机流转（纯逻辑，不含 ROS 通信）
 *
 * 状态流转：
 *   IDLE  --入队-->            ACTIVE
 *   ACTIVE --到达-->           队列非空 ? ACTIVE(下一个) : IDLE
 *   ACTIVE --卡死/失败告警-->   RETRY（重试次数 < max 时重新下发）
 *   RETRY  --次数耗尽-->        跳过该点，队列非空 ? ACTIVE : IDLE
 */
class MissionManagerCore {
  public:
    /**
     * @brief 构造函数
     * @param logger ROS 2 日志记录器
     * @param max_retry 单个任务点最大重试次数
     */
    explicit MissionManagerCore(const rclcpp::Logger& logger, int max_retry = 3);

    /**
     * @brief 接收新任务队列（整体替换当前队列并立即开始执行）
     * @param goals 任务目标点序列（世界坐标）
     */
    void setMissionQueue(const std::vector<geometry_msgs::msg::PointStamped>& goals);

    /**
     * @brief 上报当前目标已到达，推进到下一个任务点
     * @return true 队列中还有下一个任务点（节点层应读取 popCurrentGoal() 下发）；false 全部完成
     */
    bool onGoalReached();

    /**
     * @brief 上报卡死/失败告警，进入重试或跳过
     * @return true 需要重新下发当前目标（重试）；false 当前点已跳过
     */
    bool onStuckAlert();

    /**
     * @brief 取出当前应下发的目标点（仅在 setMissionQueue / onGoalReached / onStuckAlert 返回 true 后调用）
     */
    geometry_msgs::msg::PointStamped popCurrentGoal();

    /**
     * @brief 当前状态快照（节点层发布 /mission_status 用）
     */
    MissionStatus getStatus() const;

    /**
     * @brief 是否处于有任务的状态（ACTIVE 或 RETRY）
     */
    bool hasActiveMission() const;

  private:
    rclcpp::Logger logger_;
    int max_retry_;

    MissionState state_ = MissionState::IDLE;
    std::vector<geometry_msgs::msg::PointStamped> queue_;  ///< 待执行任务点（含当前未完成点）
    std::size_t current_index_ = 0;  ///< 当前执行的下标
    int retry_count_ = 0;            ///< 当前任务点已重试次数
    int finished_ = 0;               ///< 已完成计数
    int skipped_ = 0;                ///< 已跳过计数
};

}  // namespace robot

#endif  // MISSION_MANAGER_CORE_HPP_
