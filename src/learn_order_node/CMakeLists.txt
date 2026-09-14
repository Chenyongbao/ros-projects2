cmake_minimum_required(VERSION 3.16)
project(learn_order_node LANGUAGES CXX)

# ROS2 适配层只负责通信和节点生命周期，业务规则放在 learn_order_core。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(learn_order_interfaces REQUIRED)
find_package(learn_order_core REQUIRED)

# 一个节点承载两个 Service 和一个状态 Publisher。
add_executable(order_gateway_node src/order_gateway_node.cpp)
ament_target_dependencies(order_gateway_node
  rclcpp rclcpp_action geometry_msgs std_msgs std_srvs
  learn_order_interfaces learn_order_core)

add_executable(mock_navigation_server src/mock_navigation_server.cpp)
# Mock Server 只依赖 ROS2 Action 和接口包，不依赖任务队列核心库。
ament_target_dependencies(mock_navigation_server
  rclcpp rclcpp_action learn_order_interfaces)

add_executable(nav2_navigation_server src/nav2_navigation_server.cpp)
# Nav2 适配器只负责 Action 转换，不把 Nav2 类型泄漏到任务核心库。
ament_target_dependencies(nav2_navigation_server
  rclcpp rclcpp_action nav2_msgs learn_order_interfaces)

install(TARGETS order_gateway_node mock_navigation_server nav2_navigation_server
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/rest_api_gateway.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
