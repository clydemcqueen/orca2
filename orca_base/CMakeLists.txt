cmake_minimum_required(VERSION 3.5)
project(orca_base)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(ukf_DIR "${PROJECT_SOURCE_DIR}/../../../install/ukf/share/ukf/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(Eigen3)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_msgs REQUIRED)
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
# Base node
#=============

add_executable(
  base_node
  src/astar.cpp
  src/base_node.cpp
  src/map.cpp
  src/mission.cpp
  src/planner.cpp
  src/segment.cpp
  src/controller.cpp
  src/util.cpp
)

ament_target_dependencies(
  base_node
  fiducial_vlam_msgs
  orca_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  ros2_shared
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

#=============
# Filter node
#=============

add_executable(
  filter_node
  src/filter_node.cpp
  src/filter_base.cpp
  src/depth_filter.cpp
  src/four_filter.cpp
  src/pose_filter.cpp
  src/util.cpp
)

ament_target_dependencies(
  filter_node
  orca_msgs
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
# Test
#=============

add_executable(
  astar_test
  src/astar.cpp
  src/astar_test.cpp
)

ament_target_dependencies(
  astar_test
  rclcpp
  sensor_msgs
)

#=============
# Install
#=============

# Export include directory
ament_export_include_directories(include)

# Install C++ targets
install(TARGETS base_node filter_node DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install Python nodes
# PROGRAMS sets execute bits, FILES clears them
install(
  PROGRAMS
  scripts/experiments.py
  scripts/nees.py
  scripts/plot_control.py
  scripts/plot_filter.py
  scripts/plot_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()