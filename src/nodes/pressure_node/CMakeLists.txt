cmake_minimum_required(VERSION 3.8)
project(pressure_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_serial REQUIRED)
find_package(pressure_comm REQUIRED)

include_directories(
  include/pressure_node
)

add_executable(pressure_node 
  src/pressure_node.cpp 
  src/pressure_manager.cpp
)

ament_target_dependencies(pressure_node
  rclcpp
  interfaces
  common_serial
  pressure_comm
)

install(TARGETS
  pressure_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
