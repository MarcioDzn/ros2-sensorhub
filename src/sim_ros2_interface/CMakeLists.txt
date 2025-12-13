cmake_minimum_required(VERSION 3.8)
project(sim_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_pub src/simple_publisher.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

install(TARGETS
  simple_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
