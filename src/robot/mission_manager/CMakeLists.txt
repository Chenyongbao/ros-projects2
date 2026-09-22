cmake_minimum_required(VERSION 3.10)
project(mission_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 任务状态机核心库（纯逻辑层，可独立单测）
add_library(mission_manager_lib
  src/mission_manager_core.cpp)
target_include_directories(mission_manager_lib
  PUBLIC include)
ament_target_dependencies(mission_manager_lib rclcpp geometry_msgs)

# 任务管理节点可执行文件
add_executable(mission_manager_node src/mission_manager_node.cpp)
target_link_libraries(mission_manager_node mission_manager_lib)
ament_target_dependencies(mission_manager_node rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS
  mission_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
