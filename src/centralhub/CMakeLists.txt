cmake_minimum_required(VERSION 3.8)
project(centralhub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/centralhub
)

add_executable(central_node src/central_node.cpp)
target_link_libraries(central_node)
ament_target_dependencies(central_node
  rclcpp
  std_msgs
)

install(TARGETS
  central_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
