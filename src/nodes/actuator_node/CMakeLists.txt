cmake_minimum_required(VERSION 3.8)
project(actuator_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)
find_package(actuator_comm REQUIRED)

include_directories(
  include/actuator_node
)

add_executable(actuator_node 
  src/actuator_node.cpp
  src/actuator_manager.cpp)

ament_target_dependencies(actuator_node
  rclcpp
  interfaces
  common_serial
  actuator_comm
)

install(TARGETS
  actuator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
