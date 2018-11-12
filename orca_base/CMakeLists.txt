cmake_minimum_required(VERSION 3.5)
project(orca_base)
set(CMAKE_CXX_STANDARD 14)

# Emulate colcon
if($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
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
  /opt/ros/bouncy/include  # TODO
  /home/clyde/orca_ws/src/orca/install/orca_msgs/include  # TODO
)

add_executable(
  orca_base
  src/orca_base.cpp
  src/orca_motion.cpp
  src/orca_mission.cpp
)

ament_target_dependencies(
  orca_base
  "orca_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "visualization_msgs"
)

add_executable(
  orca_base_test
  src/test.cpp
)

install(TARGETS orca_base orca_base_test DESTINATION lib/orca_base)
install(FILES scripts/local_pose.py scripts/kf.py DESTINATION lib/flock2)
install(DIRECTORY launch DESTINATION share/orca_base)
install(DIRECTORY include DESTINATION include)

ament_package()