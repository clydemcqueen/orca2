cmake_minimum_required(VERSION 3.5)
project(orca_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(sim_fiducial_DIR "${PROJECT_SOURCE_DIR}/../../../install/sim_fiducial/share/sim_fiducial/cmake")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)  # Note uppercase variables in /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake
# 11-Sep-20: gazebo-config.cmake generates a developer warning. Add -Wno-dev to CLion CMake options to ignore it.
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sim_fiducial REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

#=============
# Run build_world.py to generate .yaml (vlam maps) and .world files
# Use simple.world as the marker file, if this is stale, re-run build_world.py
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/worlds")

set(MARKER_FILE "${CMAKE_CURRENT_BINARY_DIR}/worlds/simple.world")

add_custom_command(
  OUTPUT ${MARKER_FILE}
  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_world.py" "worlds"
  DEPENDS scripts/build_world.py
  COMMENT "Generate vlam maps and world files"
  VERBATIM
)

add_custom_target(generate_maps_and_worlds ALL DEPENDS ${MARKER_FILE})

#=============
# Create plugins (shared libraries) and executables
#=============

add_library(OrcaBarometerPlugin SHARED src/barometer_plugin.cpp)
ament_target_dependencies(
  OrcaBarometerPlugin
  gazebo_dev
  gazebo_ros
  geometry_msgs
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
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
  orca_shared
  rclcpp
  ros2_shared
)
ament_export_libraries(OrcaDragPlugin)

add_library(OrcaThrusterPlugin SHARED src/thruster_plugin.cpp)
ament_target_dependencies(
  OrcaThrusterPlugin
  gazebo_dev
  gazebo_ros
  orca_msgs
  orca_shared
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

## Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

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

## Install launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

## Install worlds directory
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/worlds"
  DESTINATION share/${PROJECT_NAME}
)

ament_package()