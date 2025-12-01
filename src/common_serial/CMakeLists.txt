cmake_minimum_required(VERSION 3.8)
project(common_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(
  include/common_serial
)

add_library(serial_handler
  src/serial_handler.cpp
)

install(TARGETS serial_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(serial_handler)
ament_export_include_directories("include")
ament_package()
