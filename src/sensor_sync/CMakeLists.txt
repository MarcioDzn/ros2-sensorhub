cmake_minimum_required(VERSION 3.8)
project(sensor_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  include/sensor_sync
)

add_executable(sync_node src/sync_node.cpp)

ament_target_dependencies(sync_node
  rclcpp
  interfaces
)

install(TARGETS
  sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

