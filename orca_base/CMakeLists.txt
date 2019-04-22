cmake_minimum_required(VERSION 3.5)
project(orca_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${builtin_interfaces_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${fiducial_vlam_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

#=============
# Base node
#=============

add_executable(
  base_node
  src/base_node.cpp
  src/base_context.cpp
  src/mission.cpp
  src/motion.cpp
  src/util.cpp
)

ament_target_dependencies(
  base_node
  fiducial_vlam_msgs
  orca_msgs
  nav_msgs
  rclcpp
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
  src/kf.cpp
)

ament_target_dependencies(
  filter_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_msgs
)

#=============
# Test node
#=============

add_executable(
  test_node
  src/test_node.cpp
)

ament_target_dependencies(
  test_node
  rclcpp
  sensor_msgs
)

ament_export_include_directories(include)

install(TARGETS base_node filter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_package()