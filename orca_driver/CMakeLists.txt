cmake_minimum_required(VERSION 3.5)
project(orca_driver)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

list(APPEND CMAKE_PREFIX_PATH "/home/clyde/orca_ws/install/orca_msgs/share/orca_msgs/cmake")  # TODO

find_package(ament_cmake REQUIRED)
find_package(orca_msgs REQUIRED)
#find_package(phidgets_imu REQUIRED)
#find_package(python-smbus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  /opt/ros/bouncy/include  # TODO
  /home/clyde/orca_ws/src/orca/install/orca_msgs/include  # TODO
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
