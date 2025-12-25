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
)

target_include_directories(dynamixel_link PUBLIC include)

ament_target_dependencies(dynamixel_link
  common_serial
)

install(TARGETS dynamixel_link
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
