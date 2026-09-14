#include "learn_order_core/mission_event_log.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace learn_order_core {

bool IsTerminalState(const std::string& state) {
  // 终态集合是任务状态的唯一事实来源，新增终态只在这里扩展。
  // 三个终态分别对应：正常完成（SUCCEEDED）、重试耗尽后的失败（FAILED）、
  // 操作员取消（CANCELED，含排队中直接取消与执行中 Goal 取消两种路径）。
  return state == "SUCCEEDED" || state == "FAILED" || state == "CANCELED";
}

bool HasTerminalState(const std::vector<MissionEvent>& events,
                      const std::string& mission_id) {
  // “历史视角”判断：只要该任务历史上出现过终态事件就返回 true。
  // 与 ResolveMissionStatus 的“当前视角”不同——终态后重新提交会追加
  // 新的过程事件，本函数仍然为 true，因此适合做“上一轮生命周期已结束”
  // 的去重豁免判断，而不应用来回答“任务现在是否还在跑”。
  return std::any_of(events.begin(), events.end(),
                     [&mission_id](const MissionEvent& event) {
                       return event.mission_id == mission_id && event.is_terminal;
                     });
}

void AppendMissionEvent(std::vector<MissionEvent>& events, MissionEvent event) {
  // 事件按照发生顺序追加，顺序就是任务的真实时间线。
  // 超过容量上限时丢弃最旧的事件，保证长跑进程的内存占用有界；
  // 丢弃的是最早发生的事件，时间线的相对顺序保持不变。
  // 后续可以在此处替换为持久化策略（数据库 / 文件）以保留完整历史。
  if (events.size() >= kMaxEventHistory) {
    const std::size_t overflow = events.size() - kMaxEventHistory + 1U;
    events.erase(events.begin(),
                 events.begin() + static_cast<std::ptrdiff_t>(overflow));
  }
  events.push_back(std::move(event));
}

std::vector<MissionEvent> FindMissionEvents(
    const std::vector<MissionEvent>& events, const std::string& mission_id) {
  // 空 ID 表示调用方希望查看完整历史；返回副本避免外部修改节点内部数据。
  if (mission_id.empty()) {
    return events;
  }

  // 只筛选指定任务的事件，保持原发生顺序，供 /mission_events 查询使用。
  std::vector<MissionEvent> result;
  std::copy_if(events.begin(), events.end(), std::back_inserter(result),
               [&mission_id](const MissionEvent& event) {
                 return event.mission_id == mission_id;
               });
  return result;
}

MissionStatus ResolveMissionStatus(const std::vector<MissionEvent>& events,
                                   const std::string& mission_id) {
  MissionStatus status;
  // 事件按发生顺序追加，取该任务最后一条事件即为最新状态。
  // 线性扫描足够：教学项目事件量小；如需高频查询可换成按 ID 的索引。
  const MissionEvent* latest = nullptr;
  for (const auto& event : events) {
    if (event.mission_id == mission_id) {
      latest = &event;
    }
  }
  if (latest == nullptr) {
    // 没有事件记录时返回 found=false，调用方按“未知任务”处理。
    return status;
  }
  status.found = true;
  status.state = latest->state;
  status.is_terminal = latest->is_terminal;
  status.retry_count = latest->retry_count;
  status.message = latest->message;
  // 终态时刻只描述“最后一次进入终态”；非终态任务恒为 0。
  // 例如：SUCCEEDED -> 重提 QUEUED，terminal_* 即为 0（当前不在终态）。
  if (latest->is_terminal) {
    status.terminal_sec = latest->timestamp_sec;
    status.terminal_nanosec = latest->timestamp_nanosec;
  }
  return status;
}

}  // namespace learn_order_core
