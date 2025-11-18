cmake_minimum_required(VERSION 3.8)
project(clienthub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  include/clienthub
)

add_executable(client_node 
  src/client_node.cpp 
)

ament_target_dependencies(client_node
  rclcpp
  interfaces
)

install(TARGETS
  client_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
