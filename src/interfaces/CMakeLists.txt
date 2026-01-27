cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/imu/IMUData.msg"
  "msg/imu/IMUState.msg"
  "msg/pressure/PressureUnitSensor.msg"
  "msg/pressure/PressureData.msg"
  "msg/pressure/PressureState.msg"
  "msg/actuator/Command.msg"
  "msg/actuator/ActuatorState.msg"
  "msg/SyncedSensorData.msg"
)

set(srv_files
  "srv/SetTorque.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
