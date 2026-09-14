#include "learn_order_core/mission_queue.hpp"

#include <algorithm>

namespace learn_order_core {

bool MissionQueue::Contains(const std::string& mission_id) const {
  // 检查任务是否已经在等待队列中。
  // 当前只检查等待队列；活动任务在更完整的调度器中还需单独检查。
  // 同 ID 的“排队中”与“执行中”由网关节点分别判断后再拦截重复提交。
  return std::any_of(missions_.begin(), missions_.end(),
                     [&mission_id](const auto& mission) {
                       return mission.profile.mission_id == mission_id;
                     });
}

// 入队时分配 sequence、入队并重新排序。
bool MissionQueue::Enqueue(
    const MissionProfile& profile, const int priority, std::string* message) {
  // 去重必须发生在分配 sequence 之前，拒绝任务不应消耗到达序号。
  if (Contains(profile.mission_id)) {
    if (message != nullptr) *message = "mission already queued: " + profile.mission_id;
    return false;
  }
  // sequence 只递增不复用，即使前面的任务被弹出也不会改变历史到达顺序。
  // 注意：终态任务重新提交（同 ID 重提）是允许的——
  // 此时该 ID 已不在队列，Contains 不会命中，会正常拿到新 sequence 入队。
  missions_.push_back(QueuedMission{profile, priority, next_sequence_++});
  Sort();
  if (message != nullptr) *message = "mission queued: " + profile.mission_id;
  return true;
}

void MissionQueue::Sort() {
  // 高优先级先执行；同优先级依靠 sequence 保持 FIFO。
  // 使用 stable_sort 保证排序只依赖 priority/sequence，
  // 不引入任何隐藏的顺序扰动。
  std::stable_sort(missions_.begin(), missions_.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.priority != rhs.priority) return lhs.priority > rhs.priority;
    return lhs.sequence < rhs.sequence;
  });
}

DispatchResult MissionQueue::PopNext() {
  DispatchResult result;
  // 空队列是正常业务结果，不是异常；由调用方决定是否继续等待。
  if (missions_.empty()) {
    result.message = "mission queue is empty";
    return result;
  }
  // 当前实现用 front 表示已排序队列中的下一项。
  // 任务被移出后不再属于队列；派发失败时调用方必须 Requeue，
  // 否则任务会丢失。
  result.success = true;
  result.mission = missions_.front();
  missions_.erase(missions_.begin());
  result.message = "mission dispatched: " + result.mission.profile.mission_id;
  return result;
}

bool MissionQueue::Remove(const std::string& mission_id, std::string* message) {
  // 按 ID 找到排队任务后移除，用于“取消排队任务”的同步终态路径：
  // 排队任务尚未进入 Action，取消是即时确定的，网关节点随后记录 CANCELED。
  const auto it = std::find_if(missions_.begin(), missions_.end(),
                               [&mission_id](const auto& mission) {
                                 return mission.profile.mission_id == mission_id;
                               });
  if (it == missions_.end()) {
    // 不存在时明确返回 false，并给出可读原因；
    // 调用方（网关）会把它翻译成 not_in_queue 拒绝原因。
    if (message != nullptr) *message = "mission not in queue: " + mission_id;
    return false;
  }
  // 移除不影响其他任务的相对顺序：sequence 不回收，next_sequence_ 不回退。
  missions_.erase(it);
  if (message != nullptr) *message = "mission removed from queue: " + mission_id;
  return true;
}

std::size_t MissionQueue::Size() const noexcept { return missions_.size(); }
bool MissionQueue::Empty() const noexcept { return missions_.empty(); }

}  // namespace learn_order_core
