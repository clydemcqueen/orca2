cmake_minimum_required(VERSION 3.5)
project(orca_gazebo)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(orca_base_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_base/share/orca_base/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)  # Note uppercase variables in /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_base REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

# Create plugins (shared libraries) and executables

add_library(OrcaBarometerPlugin SHARED src/barometer_plugin.cpp)
ament_target_dependencies(
  OrcaBarometerPlugin
  gazebo_dev
  gazebo_ros
  geometry_msgs
  orca_base
  orca_msgs
  rclcpp
)
ament_export_libraries(OrcaBarometerPlugin)

add_library(OrcaBuoyancyPlugin SHARED src/buoyancy_plugin.cpp)
ament_target_dependencies(
  OrcaBuoyancyPlugin
  gazebo_dev
  gazebo_ros
  rclcpp
)
ament_export_libraries(OrcaBuoyancyPlugin)

add_library(OrcaDragPlugin SHARED src/drag_plugin.cpp)
ament_target_dependencies(
  OrcaDragPlugin
  gazebo_dev
  gazebo_ros
  orca_base
  rclcpp
)
ament_export_libraries(OrcaDragPlugin)

add_library(OrcaThrusterPlugin SHARED src/thruster_plugin.cpp)
ament_target_dependencies(
  OrcaThrusterPlugin
  gazebo_dev
  gazebo_ros
  orca_base
  orca_msgs
  rclcpp
)
ament_export_libraries(OrcaThrusterPlugin)

add_library(OrcaIMUPlugin SHARED src/imu_plugin.cpp)
ament_target_dependencies(
  OrcaIMUPlugin
  gazebo_dev
  gazebo_ros
  orca_msgs
  rclcpp
  sensor_msgs
)
ament_export_libraries(OrcaIMUPlugin)

## Export dependencies
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(orca_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)

## Install targets
install(
  TARGETS
  OrcaDragPlugin
  OrcaThrusterPlugin
  OrcaBarometerPlugin
  OrcaBuoyancyPlugin
  OrcaIMUPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install various directories
install(
  DIRECTORY launch models worlds sdf
  DESTINATION share/${PROJECT_NAME}
)

## Install Python scripts
install(
  PROGRAMS src/inject_entity.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()