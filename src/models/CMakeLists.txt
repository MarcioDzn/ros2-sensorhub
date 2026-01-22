cmake_minimum_required(VERSION 3.8)
project(models)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)

add_library(models SHARED
    src/actuator.cpp
    src/pressure.cpp
    src/imu.cpp
)

target_include_directories(models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
    TARGETS models
    EXPORT modelsTargets
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION include/
)

ament_export_targets(modelsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories("include")
ament_package()

