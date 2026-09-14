# 统一安全门：/cmd_vel 全系统唯一发布者。
# 决策逻辑是 header-only 纯函数（无 rclcpp 依赖），节点只做订阅 / 发布 / 定时。
cmake_minimum_required(VERSION 3.16)
project(neobot_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 纯函数决策层：只依赖 geometry_msgs 的消息结构，不依赖 rclcpp。
add_library(cmd_vel_safety INTERFACE)
target_include_directories(cmd_vel_safety INTERFACE include)
ament_target_dependencies(cmd_vel_safety INTERFACE geometry_msgs)

add_executable(cmd_vel_safety_gate src/cmd_vel_safety_gate_node.cpp)
target_link_libraries(cmd_vel_safety_gate cmd_vel_safety)
ament_target_dependencies(cmd_vel_safety_gate
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS cmd_vel_safety_gate DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(cmd_vel_safety_test test/cmd_vel_safety_test.cpp)
  target_link_libraries(cmd_vel_safety_test cmd_vel_safety)
  ament_target_dependencies(cmd_vel_safety_test geometry_msgs)
endif()

ament_package()
