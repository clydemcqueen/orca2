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

set (mraa_INCLUDE_DIRS "/usr/include")
set (mraa_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmraa.so")

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${mraa_INCLUDE_DIRS}
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
  mraa
  orca_msgs
  rclcpp
)

#=============
# OpenCV camera component
#=============

add_library(
  opencv_camera SHARED
  src/opencv_camera.cpp
)
ament_target_dependencies(
  opencv_camera
  cv_bridge
  rclcpp
  sensor_msgs
)

# Export incantations:
# Call ament_export_interfaces()
# Reference this in the install() with an EXPORT line
# Source install/local_setup.bash after the build
ament_export_interfaces(
  opencv_camera
  HAS_LIBRARY_TARGET
)

#=============
# OpenCV camera node
#=============

add_executable(
  opencv_camera_node
  src/opencv_camera_node.cpp
)
target_link_libraries(
  opencv_camera_node
  opencv_camera
)
ament_target_dependencies(
  opencv_camera_node
  rclcpp
)

#=============
# Install
#=============

## Install C++ targets
install(
  TARGETS driver_node opencv_camera_node opencv_camera
  EXPORT opencv_camera
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

# Install Python nodes
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/bar30_node.py
  DESTINATION lib/${PROJECT_NAME}
)

## Install various directories
install(
  DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
