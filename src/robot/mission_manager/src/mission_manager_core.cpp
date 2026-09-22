// mission_manager_core.cpp
// 任务管理核心实现：多点任务队列 + IDLE/ACTIVE/RETRY 状态机流转（纯逻辑，不含 ROS 通信）

#include "mission_manager_core.hpp"

namespace robot
{

MissionManagerCore::MissionManagerCore(const rclcpp::Logger& logger, int max_retry)
  : logger_(logger), max_retry_(max_retry) {}

/**
 * @brief 接收新任务队列：整体替换并立即开始执行第一个点
 */
void MissionManagerCore::setMissionQueue(
  const std::vector<geometry_msgs::msg::PointStamped>& goals)
{
  queue_ = goals;
  current_index_ = 0;
  retry_count_ = 0;
  finished_ = 0;
  skipped_ = 0;

  if (queue_.empty()) {
    state_ = MissionState::IDLE;
    RCLCPP_WARN(logger_, "收到空任务队列，保持 IDLE");
    return;
  }
  state_ = MissionState::ACTIVE;
  RCLCPP_INFO(logger_, "接收 %zu 个任务点，开始执行第 1 个", queue_.size());
}

/**
 * @brief 当前目标已到达：推进队列
 * @return true 还有下一个任务点；false 全部完成回到 IDLE
 */
bool MissionManagerCore::onGoalReached() {
  if (state_ == MissionState::IDLE || queue_.empty()) return false;

  finished_++;
  RCLCPP_INFO(logger_, "任务点 %zu/%zu 已到达（累计完成 %d，跳过 %d）",
              current_index_ + 1, queue_.size(), finished_, skipped_);

  current_index_++;
  retry_count_ = 0;

  if (current_index_ >= queue_.size()) {
    state_ = MissionState::IDLE;
    RCLCPP_INFO(logger_, "全部任务完成！共 %d 个点（完成 %d，跳过 %d）",
                queue_.size(), finished_, skipped_);
    return false;
  }
  state_ = MissionState::ACTIVE;
  return true;
}

/**
 * @brief 卡死/失败告警：重试或跳过当前任务点
 * @return true 重试（需重新下发当前目标）；false 已跳过该点
 */
bool MissionManagerCore::onStuckAlert() {
  if (state_ == MissionState::IDLE || queue_.empty()) return false;

  retry_count_++;
  if (retry_count_ <= max_retry_) {
    state_ = MissionState::RETRY;
    RCLCPP_WARN(logger_, "任务点 %zu 卡死，第 %d/%d 次重试",
                current_index_ + 1, retry_count_, max_retry_);
    return true;
  }

  // 重试耗尽：跳过该点，继续后续任务
  skipped_++;
  RCLCPP_ERROR(logger_, "任务点 %zu 重试 %d 次仍失败，跳过", current_index_ + 1, max_retry_);
  current_index_++;
  retry_count_ = 0;

  if (current_index_ >= queue_.size()) {
    state_ = MissionState::IDLE;
    RCLCPP_ERROR(logger_, "队列执行完毕（完成 %d，跳过 %d）", finished_, skipped_);
    return false;
  }
  state_ = MissionState::ACTIVE;
  return false;
}

/**
 * @brief 取出当前应下发的目标点
 */
geometry_msgs::msg::PointStamped MissionManagerCore::popCurrentGoal() {
  return queue_.at(current_index_);
}

/**
 * @brief 当前状态快照
 */
MissionStatus MissionManagerCore::getStatus() const {
  MissionStatus s;
  s.state = state_;
  s.total = static_cast<int>(queue_.size());
  s.retry_count = retry_count_;
  s.finished = finished_;
  s.skipped = skipped_;
  s.current_index = (state_ == MissionState::IDLE || queue_.empty())
                      ? 0 : static_cast<int>(current_index_) + 1;
  return s;
}

/**
 * @brief 是否处于有任务的状态
 */
bool MissionManagerCore::hasActiveMission() const {
  return state_ != MissionState::IDLE && !queue_.empty();
}

}  // namespace robot
