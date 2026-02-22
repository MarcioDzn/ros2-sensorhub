cmake_minimum_required(VERSION 3.8)
project(time_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  include/time_node
)

add_executable(pressure_time_node src/pressure_time_node.cpp)
ament_target_dependencies(pressure_time_node rclcpp interfaces)

add_executable(actuator_time_node src/actuator_time_node.cpp)
ament_target_dependencies(actuator_time_node rclcpp interfaces)

add_executable(imu_time_node src/imu_time_node.cpp)
ament_target_dependencies(imu_time_node rclcpp interfaces)

install(TARGETS
  pressure_time_node
  actuator_time_node
  imu_time_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
