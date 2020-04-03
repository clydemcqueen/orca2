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

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

#=============
# AUV node
#=============

add_executable(
  auv_node
  src/astar.cpp
  src/auv_node.cpp
  src/controller.cpp
  src/global_planner.cpp
  src/map.cpp
  src/mission.cpp
  src/move_to_marker_planner.cpp
  src/observation_segment.cpp
  src/planner_common.cpp
  src/pose_planner.cpp
  src/pose_segment.cpp
  src/segment_common.cpp
  src/thrusters.cpp
)

ament_target_dependencies(
  auv_node
  fiducial_vlam_msgs
  geometry_msgs
  image_geometry
  nav_msgs
  orca_description
  orca_msgs
  orca_shared
  rclcpp
  rclcpp_action
  ros2_shared
  tf2
  tf2_ros
)

#=============
# ROV node
#=============

add_executable(
  rov_node
  src/rov_node.cpp
  src/thrusters.cpp
)

ament_target_dependencies(
  rov_node
  geometry_msgs
  orca_msgs
  orca_shared
  rclcpp
  rclcpp_action
  ros2_shared
  sensor_msgs
)

#=============
# Annotate image node
#=============

add_executable(
  annotate_image_node
  src/annotate_image_node.cpp
)

ament_target_dependencies(
  annotate_image_node
  cv_bridge
  orca_msgs
  orca_shared
  rclcpp
  sensor_msgs
)

#=============
# Test
#=============

add_executable(
  test_main
  src/astar.cpp
  src/astar_test.cpp
  src/pose_segment.cpp
  src/segment_common.cpp
  src/segment_test.cpp
  src/test_main.cpp
)

ament_target_dependencies(
  test_main
  fiducial_vlam_msgs
  image_geometry
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
  sensor_msgs
)

#=============
# Install
#=============

# Install C++ targets
install(
  TARGETS
  auv_node
  rov_node
  annotate_image_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
# PROGRAMS sets execute bits, FILES clears them
install(
  PROGRAMS
  scripts/experiments.py
  scripts/nees.py
  scripts/plot_auv_segments.py
  scripts/plot_control.py
  scripts/plot_filter.py
  scripts/plot_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()