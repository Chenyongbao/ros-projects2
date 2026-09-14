# 一条任务生命周期事件。
# 网关把每次状态变化（提交、派发、执行、重试、取消请求、终态）
# 同时写入内存事件历史并发布到本 Topic，供监控与后续持久化使用。
string mission_id
# 事件对应的状态名，与 MissionState.state 取值一致。
string state
# 该事件本身是否为终态事件（SUCCEEDED / FAILED / CANCELED）。
# 与 is_terminal 的区别见 MissionState：事件级终态标记只描述“这条事件”。
bool is_terminal
# 事件发生时刻（网关节点时钟），用于还原任务时间线与计算耗时。
builtin_interfaces/Time timestamp
# 记录事件时该任务已发生的重试次数（过程事件与终态事件都带）。
uint32 retry_count
# 事件的可读描述，说明发生了什么以及原因。
string message
