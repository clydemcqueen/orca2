cmake_minimum_required(VERSION 3.5)
project(orca_filter)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(ukf_DIR "${PROJECT_SOURCE_DIR}/../../../install/ukf/share/ukf/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_description REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ukf REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

#=============
# Depth node
#=============

add_executable(
  depth_node
  src/depth_node.cpp
)

ament_target_dependencies(
  depth_node
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
)

#=============
# Filter node
#=============

add_executable(
  filter_node
  src/depth_filter.cpp
  src/filter_node.cpp
  src/filter_base.cpp
  src/four_filter.cpp
  src/measurement.cpp
  src/pose_filter.cpp
)

ament_target_dependencies(
  filter_node
  orca_description
  orca_msgs
  orca_shared
  fiducial_vlam_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  ros2_shared
  tf2
  tf2_ros
  ukf
  urdf
)

#=============
# Install
#=============

# Install C++ targets
install(TARGETS depth_node filter_node DESTINATION lib/${PROJECT_NAME})

ament_package()