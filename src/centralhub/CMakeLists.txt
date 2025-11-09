cmake_minimum_required(VERSION 3.8)
project(centralhub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Detecta Raspberry Pi pelo sistema
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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/centralhub
  ${CMAKE_CURRENT_SOURCE_DIR}/../imu_bno/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../wiringpi/wiringPi
)

set(IMU_BNO_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/../imu_bno/src/bno055.cpp 
)

# Arquivos da WiringPi necessários para build no PC
if(NOT RASPBERRY_PI)
  set(WIRINGPI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../wiringpi/wiringPi/wiringPi_stub.c)
else()
  set(WIRINGPI_SOURCES "")
endif()

# bibliotecas
add_library(imu_lib
  src/libs/imu_lib.cpp
  ${IMU_BNO_SOURCES} 
)
ament_target_dependencies(imu_lib
  rclcpp
)

add_library(imu_manager
  src/managers/imu_manager.cpp
)
ament_target_dependencies(imu_manager
  rclcpp
  interfaces
)
target_link_libraries(imu_manager imu_lib)

add_executable(central_node 
  src/central_node.cpp 
  ${IMU_BNO_SOURCES} 
  ${WIRINGPI_SOURCES}
)

if(RASPBERRY_PI)
  target_link_libraries(central_node wiringPi)
endif()

ament_target_dependencies(central_node
  rclcpp
  std_msgs
  interfaces
)

target_link_libraries(central_node imu_manager)

install(TARGETS
  imu_lib
  imu_manager
  central_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
