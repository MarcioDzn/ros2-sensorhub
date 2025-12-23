cmake_minimum_required(VERSION 3.8)
project(actuator_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(dynamixel_protocol
  src/dynamixel_protocol.cpp
)

target_include_directories(dynamixel_protocol PUBLIC include)

install(TARGETS dynamixel_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(dynamixel_protocol)
ament_export_include_directories("include")
ament_package()
