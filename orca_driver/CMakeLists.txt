cmake_minimum_required(VERSION 3.5)
project(orca_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(orca_msgs REQUIRED)
#find_package(phidgets_imu REQUIRED)
#find_package(python-smbus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

#=============
# Driver node
#=============

add_executable(
  driver_node
  src/driver_node.cpp
  src/maestro.cpp
)
ament_target_dependencies(
  driver_node
  orca_msgs
  rclcpp
)

#=============
# OpenCV camera node
#=============

add_executable(
  opencv_camera_node
  src/opencv_camera_node.cpp
)
ament_target_dependencies(
  opencv_camera_node
  cv_bridge
  rclcpp
  sensor_msgs
)

## Install targets
install(
  TARGETS
  driver_node
  opencv_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install various directories
install(
  DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
