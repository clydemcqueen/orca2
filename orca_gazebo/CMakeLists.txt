cmake_minimum_required(VERSION 3.5)
project(orca_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(gazebo_dev_DIR "${PROJECT_SOURCE_DIR}/../../../install/gazebo_dev/share/gazebo_dev/cmake")
  set(gazebo_ros_DIR "${PROJECT_SOURCE_DIR}/../../../install/gazebo_ros/share/gazebo_ros/cmake")
  set(orca_base_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_base/share/orca_base/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()

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

# Set up include directories
include_directories(
  include
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${orca_base_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# Set up link directories
link_directories(${GAZEBO_LIBRARY_DIRS}) # TODO?

# Create plugins (shared libraries) and executables

add_library(OrcaBarometerPlugin SHARED src/barometer_plugin.cpp)
ament_target_dependencies(
  OrcaBarometerPlugin
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "orca_msgs"
  "rclcpp"
)
ament_export_libraries(OrcaBarometerPlugin)

add_library(OrcaBuoyancyPlugin SHARED src/buoyancy_plugin.cpp)
ament_target_dependencies(
  OrcaBuoyancyPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(OrcaBuoyancyPlugin)

add_library(OrcaDragPlugin SHARED src/drag_plugin.cpp)
ament_target_dependencies(
  OrcaDragPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(OrcaDragPlugin)

add_library(OrcaThrusterPlugin SHARED src/thruster_plugin.cpp)
ament_target_dependencies(
  OrcaThrusterPlugin
  "gazebo_dev"
  "gazebo_ros"
  "orca_msgs"
  "rclcpp"
)
ament_export_libraries(OrcaThrusterPlugin)

add_library(OrcaGPSPlugin SHARED src/gps_plugin.cpp)
ament_target_dependencies(
  OrcaGPSPlugin
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
ament_export_libraries(OrcaGPSPlugin)

add_library(OrcaGroundTruthPlugin SHARED src/ground_truth_plugin.cpp)
ament_target_dependencies(
  OrcaGroundTruthPlugin
  "gazebo_dev"
  "gazebo_ros"
  "nav_msgs"
  "rclcpp"
)
ament_export_libraries(OrcaGroundTruthPlugin)

add_library(OrcaIMUPlugin SHARED src/imu_plugin.cpp)
ament_target_dependencies(
  OrcaIMUPlugin
  "gazebo_dev"
  "gazebo_ros"
  "orca_msgs"
  "rclcpp"
  "sensor_msgs"
)
ament_export_libraries(OrcaIMUPlugin)

add_executable(orca_gazebo_test src/test.cpp)

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
  OrcaGPSPlugin
  OrcaBuoyancyPlugin
  OrcaIMUPlugin
  OrcaGroundTruthPlugin
  orca_gazebo_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install worlds
install(
  DIRECTORY worlds
  DESTINATION share/orca_gazebo/
)

ament_package()