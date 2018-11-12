cmake_minimum_required(VERSION 3.5)
project(orca_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  orca_msgs
  "msg/Barometer.msg"
  "msg/Battery.msg"
  "msg/Control.msg"
  "msg/Leak.msg"
  "msg/Proc.msg"
  DEPENDENCIES std_msgs
)

ament_package()