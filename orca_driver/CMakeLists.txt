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
  set(fiducial_vlam_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam/share/fiducial_vlam/cmake")
  set(gscam_DIR "${PROJECT_SOURCE_DIR}/../../../install/gscam/share/gscam/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# If we're running fiducial_vlam_sam instead of fiducial_vlam, set this to TRUE
# This shouldn't be required... a bug in the GTSAM CMakeLists.txt?
set(RUN_GTSAM FALSE)

find_package(ament_cmake REQUIRED)
find_package(br_ms5837 REQUIRED)
find_package(fiducial_vlam REQUIRED)
find_package(gscam REQUIRED)
if (RUN_GTSAM)
  find_package(GTSAM REQUIRED)
endif ()
find_package(orca_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

set(mraa_INCLUDE_DIRS "/usr/include")
set(mraa_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmraa.so")
set(mraa_FOUND true)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
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
  br_ms5837
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
# Composite gscam and fiducial_vlam main
#=============

add_executable(
  gscam_vloc_main
  src/gscam_vloc_main.cpp
)
ament_target_dependencies(
  gscam_vloc_main
  fiducial_vlam
  gscam
  rclcpp
)

# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
target_link_libraries(gscam_vloc_main gscam::gscam_node fiducial_vlam::vloc_node)

if (RUN_GTSAM)
  target_link_libraries(gscam_vloc_main gtsam)
endif ()

#=============
# Install
#=============

# Install C++ targets
install(
  TARGETS driver_node test_node gscam_vloc_main
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

# Install Python nodes
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/start_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch cfg maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
