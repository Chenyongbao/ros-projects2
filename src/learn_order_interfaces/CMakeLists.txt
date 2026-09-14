cmake_minimum_required(VERSION 3.16)
project(learn_order_interfaces)

# 接口包负责把 .srv 文件生成 C++/Python 类型支持代码。
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 当前只生成统一订单入口 SubmitOrder。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionState.msg"
  "msg/MissionEvent.msg"
  "srv/SubmitOrder.srv"
  "srv/GetMissionEvents.srv"
  "srv/CancelMission.srv"
  "srv/GetMissionStatus.srv"
  # Action 定义由 Goal / Result / Feedback 三段组成。
  "action/NavigateSequence.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# NavigateSequence 的目标点使用 geometry_msgs/PoseStamped。
# 状态与事件消息的时间戳使用 builtin_interfaces/Time。
ament_export_dependencies(builtin_interfaces geometry_msgs)
# 下游包运行时需要 rosidl_default_runtime 才能使用生成的接口。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
