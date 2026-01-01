cmake_minimum_required(VERSION 3.8)
project(pressure_comm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(common_serial REQUIRED)

add_library(pressure_comm
  src/core/pressure_factory.cpp
  src/base/base_controller.cpp
  src/base/pressure_link.cpp
)

target_include_directories(pressure_comm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/base
)

target_compile_features(pressure_comm PUBLIC cxx_std_20)

ament_target_dependencies(pressure_comm
  common_serial
)

install(TARGETS pressure_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(pressure_comm)
ament_export_include_directories("include")
ament_export_dependencies(common_serial)
ament_package()
