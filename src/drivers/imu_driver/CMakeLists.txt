cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(imu_comm REQUIRED)

include_directories(
  include/imu_driver
)

# IMU NODE
add_executable(imu_node 
  src/imu_node.cpp 
  src/imu_manager.cpp
)

ament_target_dependencies(imu_node
  rclcpp
  interfaces
  imu_comm
)
  
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
