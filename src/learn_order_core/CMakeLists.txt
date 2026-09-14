cmake_minimum_required(VERSION 3.16)
project(learn_order_core LANGUAGES CXX)

# 核心库使用 C++17，但不依赖 ROS2，便于独立测试和复用。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# 将订单路由、任务模型、预检和队列编译成一个可被 ROS 节点链接的库。
add_library(${PROJECT_NAME}
  src/order_router.cpp
  src/mission_profile.cpp
  src/mission_preflight.cpp
  src/mission_queue.cpp
  src/mission_event_log.cpp
)
# PUBLIC 表示下游 learn_order_node 也能获得这些头文件搜索路径。
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装库和导出文件，使其他 ament 包可以 find_package 后使用它。
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# 只有开启 BUILD_TESTING 时才编译核心库的 GoogleTest。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(learn_order_core_test test/learn_order_core_test.cpp)
  target_link_libraries(learn_order_core_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
