cmake_minimum_required(VERSION 3.10)
project(ekf_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# EKF 滤波核心库（纯数学，无 ROS 消息依赖，可独立单测）
add_library(ekf_lib
  src/ekf_core.cpp)
target_include_directories(ekf_lib PUBLIC include)

# EKF 定位节点可执行文件
add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ekf_lib)
ament_target_dependencies(ekf_node rclcpp nav_msgs sensor_msgs geometry_msgs)

install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
