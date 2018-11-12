cmake_minimum_required(VERSION 3.5)
project(orca_topside)
set(CMAKE_CXX_STANDARD 14)

# Example: https://github.com/ros2/rviz/blob/ros2/rviz_default_plugins/CMakeLists.txt

# Emulate colcon
if($ENV{CLION_IDE})
  set(orca_base_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_base/share/orca_base/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()

## Find everything we need in ROS
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orca_base REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz_common REQUIRED)  # TODO get rid of class_loader warning
find_package(rviz_ogre_vendor REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Debugging
message("orca_base_INCLUDE_DIRS " "${orca_base_INCLUDE_DIRS}")  # TODO why empty?
message("orca_msgs_INCLUDE_DIRS " "${orca_msgs_INCLUDE_DIRS}")
message("rviz_common_INCLUDE_DIRS " "${rviz_common_INCLUDE_DIRS}")
message("geometry_msgs_INCLUDE_DIRS " "${geometry_msgs_INCLUDE_DIRS}")
message("tf2_ros_INCLUDE_DIRS " "${tf2_ros_INCLUDE_DIRS}")
message("rviz_ogre_vendor_INCLUDE_DIRS " "${rviz_ogre_vendor_INCLUDE_DIRS}")  # TODO why empty?

## Set up include directories
include_directories(
  include
  ${orca_base_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${rviz2_INCLUDE_DIRS}
  /home/clyde/rviz_ws/install/opt/rviz_ogre_vendor/include/OGRE  # TODO
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

set(QT_LIBRARIES Qt5::Widgets)  # TODO do I still need this?
find_package(Qt5GStreamer REQUIRED)  # Depends on Boost TODO does this have to be here?

## More include directories
include_directories(${QTGSTREAMER_INCLUDES})

## Qt MOC can't find .h files if they're in a different directory; as a work-around add them here
add_library(
  orca_topside
  src/orca_panel.cpp
  src/video_panel.cpp
  include/orca_topside/orca_panel.hpp
  include/orca_topside/video_panel.hpp
)

ament_target_dependencies(
  orca_topside
  "orca_base"
  "orca_msgs"
  "rclcpp"
  "rclpy"
  "rviz2"
)

target_link_libraries(orca_topside ${QT_LIBRARIES} ${QTGSTREAMER_UI_LIBRARIES})  # TODO add libraries, or is this the right thing?

ament_package()