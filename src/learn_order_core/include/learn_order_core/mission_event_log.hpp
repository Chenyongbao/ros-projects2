#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace learn_order_core {

// 内存事件历史的容量上限：超过后丢弃最旧事件，保证长跑进程内存有界。
// 只影响 /mission_events 的可回溯范围，不影响当前任务的调度与状态判断。
inline constexpr std::size_t kMaxEventHistory = 1000;

// 判断状态字符串是否属于任务终态集合（SUCCEEDED / FAILED / CANCELED）。
// 这是整个系统判断“任务生命周期是否结束”的唯一事实来源：
//   - 节点记录事件时用它填充 MissionEvent::is_terminal；
//   - 发布状态 Topic 时用它填充 MissionState::is_terminal；
//   - 后续如果要新增终态（例如 TIMED_OUT），只需在这里扩展。
bool IsTerminalState(const std::string& state);

// 一条任务生命周期事件。核心库只保存纯 C++ 数据，ROS 节点决定如何发布它。
//
// 字段顺序约定：新增字段必须追加在末尾，
// 因为既有代码（节点、测试）存在按位置聚合初始化的调用，
// 例如 MissionEvent{"order_a", "QUEUED", "queued", 0}。
struct MissionEvent {
  std::string mission_id;  // 事件所属任务的内部 ID。
  std::string state;       // 事件对应的状态名（QUEUED / RUNNING / SUCCEEDED ...）。
  std::string message;     // 可读描述，说明发生了什么以及原因。
  int retry_count = 0;     // 记录事件时该任务已发生的重试次数。
  // 是否为终态事件；节点记录时由 IsTerminalState 填充，保持查询侧无需重复判断。
  bool is_terminal = false;
  // 事件发生的节点时钟时间（秒 / 纳秒），用于终态时刻与超时判断。
  std::uint32_t timestamp_sec = 0;
  std::uint32_t timestamp_nanosec = 0;
};

// 任务的最新状态快照，由事件历史解析而来（取该任务最近一条事件）。
// 它是 /mission_status 查询 Service 的纯 C++ 侧返回类型，
// 不依赖 ROS，方便在单元测试中直接验证解析规则。
struct MissionStatus {
  // 是否找到该任务的任何事件；false 时其余字段均无有效语义。
  bool found = false;
  // 最近一条事件的 state；found=false 时为空。
  std::string state;
  // 最近一条事件是否已进入终态。
  // 注意语义：终态后重新提交会追加新的过程事件，此时这里为 false，
  // 与 HasTerminalState 的“历史上出现过终态”判断不同。
  bool is_terminal = false;
  // 终态时刻；非终态为 0（不代表任务未开始，只是尚未终态）。
  std::uint32_t terminal_sec = 0;
  std::uint32_t terminal_nanosec = 0;
  // 最近一条事件记录的重试次数。
  int retry_count = 0;
  // 最近一条事件的可读 message。
  std::string message;
};

// 判断某个 mission_id 历史上是否进入过终态（存在终态事件）。
// 与 ResolveMissionStatus().is_terminal 的区别：
//   - 本函数是“历史视角”：一旦见过终态就始终为 true，重提不会清除；
//   - ResolveMissionStatus 是“当前视角”：取最新事件，重提后回到过程状态。
// 本判断可用于入队去重豁免等需要“该 ID 上一轮生命周期已结束”的场景。
bool HasTerminalState(const std::vector<MissionEvent>& events,
                      const std::string& mission_id);

// 追加一条事件到内存历史。
// 事件按发生顺序保存；超过 kMaxEventHistory 时自动丢弃最旧的事件。
void AppendMissionEvent(std::vector<MissionEvent>& events, MissionEvent event);

// 按任务 ID 查询事件。
// mission_id 为空时返回全部事件，返回值是副本，避免调用方修改内部历史。
std::vector<MissionEvent> FindMissionEvents(
    const std::vector<MissionEvent>& events, const std::string& mission_id);

// 从事件历史解析某个任务的最新状态（取该任务最近一条事件）。
// 规则：
//   - 事件按发生顺序追加，所以“最后一条”就是“最新状态”；
//   - 找不到任何事件时返回 found=false 的空快照；
//   - 只有最新事件本身是终态时，terminal_* 字段才有效，否则为 0。
MissionStatus ResolveMissionStatus(const std::vector<MissionEvent>& events,
                                   const std::string& mission_id);

}  // namespace learn_order_core
