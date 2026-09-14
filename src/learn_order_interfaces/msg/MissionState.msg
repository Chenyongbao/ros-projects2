# 任务状态的结构化表示，避免下游解析 std_msgs/msg/String。
# 由网关在每次状态变化时发布：提交入队、派发、执行、重试、取消、终态。
string state
# 当前状态是否为终态（SUCCEEDED / FAILED / CANCELED）。
# 过程状态（QUEUED / DISPATCHED / RUNNING / RETRY_QUEUED / CANCEL_REQUESTED）为 false。
# 下游可直接用它判断任务生命周期是否结束，无需自己维护终态字符串集合。
bool is_terminal
# 该状态发布时刻（网关节点时钟）。
# 用途：下游做执行超时判断、事件排序，以及排查状态滞留问题。
builtin_interfaces/Time timestamp
# 状态所属任务的内部 ID。
string mission_id
# 发布时刻的等待队列长度（不含正在执行的任务）。
uint32 queue_size
# 该任务已发生的重试次数；无任务或新任务为 0。
uint32 retry_count
# 与字符串 Topic 内容一致的可读描述：state / mission_id / queue_size 的拼接。
string message
