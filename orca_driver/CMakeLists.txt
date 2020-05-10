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
  set(br_ms5837_DIR "${PROJECT_SOURCE_DIR}/../../../install/br_ms5837/share/br_ms5837/cmake")
  set(h264_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/h264_msgs/share/h264_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

find_package(ament_cmake REQUIRED)
find_package(br_ms5837 REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)

# Find ffmpeg libraries for h264_cam_node
find_package(PkgConfig)
pkg_check_modules(PC_AVDEVICE REQUIRED libavdevice>=57.10.100)
pkg_check_modules(PC_AVFORMAT REQUIRED libavformat>=57.83.100)
pkg_check_modules(PC_AVCODEC REQUIRED libavcodec>=57.107.100)
pkg_check_modules(PC_AVUTIL REQUIRED libavutil>=55.78.100)

# Hack for MRAA
set(mraa_INCLUDE_DIRS "/usr/include")
set(mraa_FOUND true)

# x86 vs ARM -- this works on Linux
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Detected aarch64")
  set(mraa_LIBRARIES "/usr/lib/aarch64-linux-gnu/libmraa.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPROCESSOR_AARCH64")
else()
  message(STATUS "Default to x86_64")
  set(mraa_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmraa.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPROCESSOR_X86_64")
endif()

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

#=============
# Barometer node
#=============

add_executable(
  barometer_node
  src/barometer_node.cpp
)
ament_target_dependencies(
  barometer_node
  br_ms5837
  mraa
  orca_msgs
  rclcpp
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
# Test node -- send simple Control messages to the driver
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
# H264 camera node
#=============

add_executable(
  h264_cam_node
  src/h264_cam_node.cpp
)
ament_target_dependencies(
  h264_cam_node
  h264_msgs
  rclcpp
  ros2_shared # TODO maybe
)

target_link_libraries(
  h264_cam_node
  ${PC_AVDEVICE_LIBRARIES}
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
)

#=============
# Install
#=============

# Install C++ targets
install(
  TARGETS barometer_node driver_node test_node h264_cam_node
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/start_driver.sh
  scripts/start_fcam.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch cfg maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
