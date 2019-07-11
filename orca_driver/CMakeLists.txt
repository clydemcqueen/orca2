cmake_minimum_required(VERSION 3.5)
project(orca_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  message("Running inside CLion")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

find_package(ament_cmake REQUIRED)
find_package(orca_msgs REQUIRED)
#find_package(phidgets_imu REQUIRED)
#find_package(python-smbus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

set(mraa_INCLUDE_DIRS "/usr/include")
set(mraa_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmraa.so")

include_directories(
  include
  ${mraa_INCLUDE_DIRS}
  ${orca_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

#=============
# Driver node
#=============

add_executable(
  driver_node
  src/driver_node.cpp
  src/maestro.cpp
)
ament_target_dependencies(
  driver_node
  mraa
  orca_msgs
  rclcpp
  ros2_shared
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
  orca_msgs
  rclcpp
)

#=============
# Install
#=============

## Install C++ targets
install(
  TARGETS driver_node test_node
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

# Install Python nodes
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/bar30_node.py
  scripts/start_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

## Install various directories
install(
  DIRECTORY launch cfg maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
