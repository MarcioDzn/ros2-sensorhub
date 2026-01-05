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

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(imu_driver
  src/bno055imu.cpp
  src/driver/bno055.cpp
)

target_include_directories(imu_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(imu_driver PUBLIC cxx_std_20)

if(RASPBERRY_PI)
  message(STATUS "Compilando para Raspberry Pi: usando wiringPi")
  target_link_libraries(imu_driver PRIVATE wiringPi)
else()
  message(STATUS "Fora do Raspberry Pi: usando wiringPi stub")
  target_sources(imu_driver
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../wiringpi/wiringPi/wiringPi_stub.c)
endif()
  
install(TARGETS imu_driver
  EXPORT export_imu_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_imu_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
