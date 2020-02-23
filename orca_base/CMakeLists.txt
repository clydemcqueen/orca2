cmake_minimum_required(VERSION 3.5)
project(orca_base)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_description_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_description/share/orca_description/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(ukf_DIR "${PROJECT_SOURCE_DIR}/../../../install/ukf/share/ukf/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
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
# Base node
#=============

add_executable(
  base_node
  src/astar.cpp
  src/base_node.cpp
  src/fp.cpp
  src/local_planner.cpp
  src/map.cpp
  src/move_to_marker_planner.cpp
  src/mission.cpp
  src/global_planner.cpp
  src/segment.cpp
  src/controller.cpp
)

ament_target_dependencies(
  base_node
  cv_bridge
  fiducial_vlam_msgs
  image_geometry
  nav_msgs
  orca_description
  orca_msgs
  orca_shared
  rclcpp
  rclcpp_action
  ros2_shared
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
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

# Install C++ targets
install(TARGETS base_node DESTINATION lib/${PROJECT_NAME})

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