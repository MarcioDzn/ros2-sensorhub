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
find_package(actuator_driver REQUIRED)

include_directories(
  include/actuator_node
)

add_library(actuator_node_lib SHARED
  src/actuator_node.cpp
  src/node_manager.cpp
  src/control/model/actuator_manager.cpp
  src/control/node/parameter_manager.cpp
  src/model/actuator.cpp
)

ament_target_dependencies(actuator_node_lib
  rclcpp
  interfaces
  common_serial
  actuator_driver
)

add_executable(actuator_node src/main.cpp)
target_link_libraries(actuator_node actuator_node_lib)

install(TARGETS
  actuator_node
  actuator_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_node
    test/integration_test.cpp
  )
  target_include_directories(test_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_node
    actuator_node_lib
  )
endif()

ament_package()
