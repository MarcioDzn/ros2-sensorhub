cmake_minimum_required(VERSION 3.8)
project(imu_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(imu_comm
  src/bno055imu.cpp
  src/driver/bno055.cpp
)

target_include_directories(imu_comm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver
)

target_compile_features(imu_comm PUBLIC cxx_std_20)

install(TARGETS imu_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(imu_comm)
ament_export_include_directories("include")

ament_package()
