cmake_minimum_required(VERSION 3.5)
project(orca_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(gscam REQUIRED)
#find_package(image_transport REQUIRED)
#find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(stereo_image_proc REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  /opt/ros/bouncy/include  # TODO
  ${OpenCV_INCLUDE_DIRS}
)

#add_executable(altitude src/altitude.cpp)
#ament_target_dependencies(altitude ${OpenCV_LIBRARIES})

#add_executable(orca_mono_odom src/mono_odometry.cpp)
#ament_target_dependencies(orca_mono_odom ${OpenCV_LIBRARIES})

#add_executable(orca_stereo_odom src/stereo_odometry.cpp src/stereo_image.cpp src/util.cpp src/test.cpp)
#ament_target_dependencies(orca_stereo_odom ${OpenCV_LIBRARIES})

ament_package()