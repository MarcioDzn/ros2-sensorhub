cmake_minimum_required(VERSION 3.8)
project(actuator_comm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(common_serial REQUIRED)

add_library(dynamixel_link
  src/link/dynamixel_link.cpp
  src/protocol/dynamixel_protocol.cpp
)

target_include_directories(dynamixel_link PUBLIC include)

ament_target_dependencies(dynamixel_link
  common_serial
)

add_executable(test_dynamixel src/main.cpp)
target_link_libraries(test_dynamixel dynamixel_link)

install(TARGETS dynamixel_link test_dynamixel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(dynamixel_link)
ament_export_include_directories("include")
ament_package()
