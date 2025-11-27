cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# detecta Raspberry Pi pelo sistema
if(EXISTS "/proc/cpuinfo")
  file(READ "/proc/cpuinfo" CPUINFO)
  if(CPUINFO MATCHES "BCM|Raspberry")
    set(RASPBERRY_PI TRUE)
  else()
    set(RASPBERRY_PI FALSE)
  endif()
else()
  set(RASPBERRY_PI FALSE)
endif()

# encontra as dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/imu_driver
)

add_library(imu_lib
  src/imu_lib.cpp
  src/bno055.cpp
)

ament_target_dependencies(imu_lib
  rclcpp
)

# Arquivos da WiringPi necessários para build no PC
if(NOT RASPBERRY_PI)
  set(WIRINGPI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../wiringpi/wiringPi/wiringPi_stub.c)
else()
  set(WIRINGPI_SOURCES "")
endif()

# IMU NODE
add_executable(imu_node src/imu_node.cpp 
  src/imu_lib.cpp
  ${WIRINGPI_SOURCES}
)

target_link_libraries(imu_node imu_lib)

if(RASPBERRY_PI)
  target_link_libraries(imu_node wiringPi)
endif()

ament_target_dependencies(imu_node
  rclcpp
  interfaces
)
  
install(TARGETS
  imu_lib
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
