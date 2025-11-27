cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/imu_driver
)

add_library(imu_lib
  src/bno055.cpp
)

# Arquivos da WiringPi necessários para build no PC
if(NOT RASPBERRY_PI)
  set(WIRINGPI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../wiringpi/wiringPi/wiringPi_stub.c)
else()
  set(WIRINGPI_SOURCES "")
endif()

install(TARGETS
  imu_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
