cmake_minimum_required(VERSION 3.10)
project(sensor_simulator)

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

add_executable(sensor_simulator src/sensor_simulator.cpp)
target_include_directories(sensor_simulator PUBLIC include)
ament_target_dependencies(sensor_simulator rclcpp nav_msgs sensor_msgs geometry_msgs)

install(TARGETS
  sensor_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
