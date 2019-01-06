cmake_minimum_required(VERSION 3.5)
project(orca_topside)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(orca_base_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_base/share/orca_base/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()

# Follow example: https://github.com/ros2/rviz/blob/ros2/rviz_default_plugins/CMakeLists.txt

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## Find everything we need
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orca_base REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(pluginlib REQUIRED)  # TODO need?
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz_common REQUIRED) # TODO fix class_loader warning
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Debugging
message("rviz_common_INCLUDE_DIRS " "${rviz_common_INCLUDE_DIRS}")
message("rviz_rendering_INCLUDE_DIRS " "${rviz_rendering_INCLUDE_DIRS}")
message("OGRE_INCLUDE_DIRS " "${OGRE_INCLUDE_DIRS}")
message("Qt5Widgets_INCLUDE_DIRS " "${Qt5Widgets_INCLUDE_DIRS}")
message("pluginlib_INCLUDE_DIRS " "${pluginlib_INCLUDE_DIRS}")

## Set up include directories
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${orca_base_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS}  # TODO need?
  ${OGRE_INCLUDE_DIRS}  # TODO need?
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Qt5GStreamer stuff
#set(QT_LIBRARIES Qt5::Widgets)  # TODO
#find_package(Qt5GStreamer REQUIRED)  # Depends on Boost TODO
#include_directories(${QTGSTREAMER_INCLUDES})

## Qt MOC can't find .h files if they're in a different directory; as a work-around add them here
add_library(
  orca_topside SHARED
  src/orca_panel.cpp
  #src/video_panel.cpp
  include/orca_topside/orca_panel.hpp
  #include/orca_topside/video_panel.hpp
)

ament_target_dependencies(
  orca_topside
  "geometry_msgs"  # TODO are quotes required?
  "orca_base"
  "orca_msgs"
  "rclcpp"
  "rviz_common"
  "rviz_ogre_vendor"
  "tf2_msgs"
  "tf2_ros"
)

target_link_libraries(
  orca_topside
  rviz_common::rviz_common
)

ament_export_libraries(orca_topside)

install(
  TARGETS orca_topside
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install cfg directory
install(
  DIRECTORY
  cfg
  DESTINATION share/orca_topside
)

ament_package()