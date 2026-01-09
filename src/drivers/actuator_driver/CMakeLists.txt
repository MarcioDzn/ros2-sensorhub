cmake_minimum_required(VERSION 3.8)
project(actuator_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(common_serial REQUIRED)


add_library(actuator_driver
  src/core/actuator_factory.cpp
  src/dynamixel/dynamixel_controller.cpp
  src/dynamixel/dynamixel_link.cpp
  src/dynamixel/dynamixel_protocol.cpp
)

target_include_directories(actuator_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel
)

target_compile_features(actuator_driver PUBLIC cxx_std_20)

ament_target_dependencies(actuator_driver
  common_serial
)


install(TARGETS actuator_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_link
    test/link_test.cpp
  )
  target_include_directories(test_link PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_link
    actuator_driver
  )

  ament_add_gmock(test_protocol
    test/protocol_test.cpp
  )
  target_include_directories(test_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_protocol
    actuator_driver
  )

  ament_add_gmock(test_serial_tty
    test/integration/serial_tty_test.cpp
  )
  target_include_directories(test_serial_tty PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_serial_tty
    actuator_driver
  )
endif()

ament_export_libraries(actuator_driver)
ament_export_include_directories("include")
ament_export_dependencies(common_serial)
ament_package()
