cmake_minimum_required(VERSION 3.8)
project(sync_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include/sync_node
)

add_library(sync_node_lib SHARED
  src/sync_node.cpp
)

ament_target_dependencies(sync_node_lib
  rclcpp
  interfaces
  message_filters
)

add_executable(sync_node src/main.cpp)
target_link_libraries(sync_node sync_node_lib)

install(TARGETS
  sync_node
  sync_node_lib
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
    sync_node_lib
  )
endif()

ament_package()

