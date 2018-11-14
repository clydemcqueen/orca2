cmake_minimum_required(VERSION 3.5)
project(orca_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(CMAKE_MODULE_PATH "/opt/ros/bouncy/share/fastrtps_cmake_module/cmake/Modules")
  message("Set CMAKE_MODULE_PATH=" "${CMAKE_MODULE_PATH}")
  find_package(FastRTPS)  # TODO this still results in errors in CLion
endif()

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