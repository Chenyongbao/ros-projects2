#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "learn_order_core/mission_profile.hpp"

namespace learn_order_core {

// 进入队列后的任务快照。
// sequence 是单调递增的到达序号，用于保证同优先级任务先来先服务。
// 重试任务重新入队时会拿到新的 sequence，但 priority 保持原值。
struct QueuedMission {
  MissionProfile profile;     // 统一任务模型：mission_id、坐标系与 waypoint 列表。
  int priority = 0;           // 从 Service 请求继承的优先级，数值越大越先执行。
  std::uint64_t sequence = 0; // 入队时分配的单调递增到达序号，只增不复用。
};

// PopNext 的返回值。mission 只有在 success=true 时才有有效语义。
// 任务出队/派发时的“结果对象”。
struct DispatchResult {
  bool success = false; // 是否成功取到了任务（队列为空时为 false）。
  std::string message;  // 成功时的成功描述或失败时的错误信息。
  QueuedMission mission; // 被取出的任务快照；success=false 时无语义。
};

// 优先级等待队列：按“priority 降序、sequence 升序”维护待执行任务。
// 教学实现用 vector + 全量排序，规模小、逻辑直观；
// 生产规模应换成堆（priority_queue）或分段索引。
class MissionQueue {
public:
  // 判断某个任务 ID 是否已经在等待队列中。
  // 注意：只检查等待队列，不检查“正在执行”的任务；
  // 完整的“同 ID 唯一”判断由网关节点结合活动任务状态完成。
  bool Contains(const std::string &mission_id) const;
  // 检查重复后分配 sequence、入队并重新排序。
  // 重复入队（同 ID 已在队列）返回 false 且不消耗 sequence。
  bool Enqueue(const MissionProfile &profile, int priority,
               std::string *message);
  // 取出当前最高优先级的任务；队列为空时返回 success=false。
  // 取出的任务从此不再属于队列；派发失败时必须由调用方 Requeue。
  DispatchResult PopNext();
  // 按任务 ID 从等待队列移除（取消排队任务）；不存在时返回 false。
  // 被移除任务的 sequence 不会回收，next_sequence_ 不回退，
  // 因此不影响其他任务的相对到达顺序。
  bool Remove(const std::string& mission_id, std::string* message);
  // 当前排队任务数量（不含正在执行的任务）。
  std::size_t Size() const noexcept;
  // 队列是否为空，供自动派发定时器做快速短路判断。
  bool Empty() const noexcept;

private:
  // 排序规则：priority 降序，sequence 升序。
  void Sort();

  // 使用 vector 保存教学项目中的待执行任务。
  std::vector<QueuedMission> missions_;
  // 下一次入队使用的到达序号，不能因 PopNext 而回退。
  std::uint64_t next_sequence_ = 0;
};

} // namespace learn_order_core
