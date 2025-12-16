cmake_minimum_required(VERSION 3.8)
project(actuator_driver)

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
  include/actuator_driver
)

add_library(actuator_manager
  src/actuator_manager.cpp
)

ament_target_dependencies(actuator_manager
  common_serial
)

add_executable(actuator_node src/actuator_node.cpp
  src/actuator_manager.cpp
)

target_link_libraries(actuator_node actuator_manager)

ament_target_dependencies(actuator_node
  rclcpp
  interfaces
  common_serial
)

install(TARGETS
  actuator_node
  actuator_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
