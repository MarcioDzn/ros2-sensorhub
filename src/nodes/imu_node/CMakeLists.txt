cmake_minimum_required(VERSION 3.8)
project(imu_node)

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
  include/imu_node
)

# IMU NODE
add_executable(imu_node 
  src/imu_node.cpp 
  src/imu_manager.cpp
  src/driver/common/bno055.cpp
  src/driver/bno055imu.cpp
)

target_include_directories(imu_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(RASPBERRY_PI)
  message(STATUS "Compilando para Raspberry Pi: usando wiringPi")
  target_link_libraries(imu_node PRIVATE wiringPi)
else()
  message(STATUS "Fora do Raspberry Pi: usando wiringPi stub")
  target_sources(imu_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../wiringpi/wiringPi/wiringPi_stub.c)
endif()

ament_target_dependencies(imu_node
  rclcpp
  interfaces
)
  
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
