cmake_minimum_required(VERSION 3.5)
project(orca_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()


find_package(ament_cmake REQUIRED)
find_package(orca_msgs REQUIRED)
#find_package(phidgets_imu REQUIRED)
#find_package(python-smbus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  find_package(orca_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
)

add_executable(
  orca_driver
  src/orca_driver.cpp
  src/maestro.cpp
)
ament_target_dependencies(
  orca_driver
  "orca_msgs"
  "rclcpp"
  "rclpy"
  "std_msgs"
)

ament_package()
