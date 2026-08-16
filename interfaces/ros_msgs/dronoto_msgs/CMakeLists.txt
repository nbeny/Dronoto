cmake_minimum_required(VERSION 3.16)
project(dronoto_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlSetpoint.msg"
  "msg/SafetyState.msg"
  "msg/SafetyEvent.msg"
  "msg/VehicleTelemetry.msg"
  "srv/TriggerFailsafe.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
