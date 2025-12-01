cmake_minimum_required(VERSION 3.8)
project(manipulator_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)

include_directories(
  include/manipulator_driver
)

add_library(manipulator_manager
  src/manipulator_manager.cpp
)

add_executable(manipulator_node src/manipulator_node.cpp
  src/manipulator_manager.cpp
)

target_link_libraries(manipulator_node manipulator_manager)

ament_target_dependencies(manipulator_node
  rclcpp
  interfaces
  common_serial
)

install(TARGETS
  manipulator_node
  manipulator_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
