cmake_minimum_required(VERSION 3.8)
project(af_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/BusServoState.msg"
  "msg/ButtonState.msg"
  "msg/BuzzerState.msg"
  "msg/GetBusServoCmd.msg"
  "msg/GetPWMServoCmd.msg"
  "msg/LedState.msg"
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
  "msg/MotorState.msg"
  "msg/MotorsState.msg"
  "msg/OLEDState.msg"
  "msg/PWMServoState.msg"
  "msg/RGBState.msg"
  "msg/RGBStates.msg"
  "msg/Sbus.msg"
  "msg/ServoPosition.msg"
  "msg/ServosPosition.msg"
  "msg/SetBusServoState.msg"
  "msg/SetPWMServoState.msg"
)

set(srv_files
  "srv/GetBusServoState.srv"
  "srv/GetPWMServoState.srv"
  "srv/ValidateCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
