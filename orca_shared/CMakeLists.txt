cmake_minimum_required(VERSION 3.5)
project(orca_shared)

#=============
# Setup
#=============

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(astar_DIR "${PROJECT_SOURCE_DIR}/../../../install/astar/share/astar/cmake")
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(astar REQUIRED)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Local includes
include_directories(
  include
)

#=============
# Library
#=============

set(ORCA_SHARED_LIB "orca_shared")

add_library(
  ${ORCA_SHARED_LIB} SHARED
  src/baro.cpp
  src/fp.cpp
  src/geometry.cpp
  src/model.cpp
  src/mw.cpp
  src/observer.cpp
  src/pwm.cpp
  src/util.cpp
)

ament_target_dependencies(
  ${ORCA_SHARED_LIB}
  astar
  fiducial_vlam_msgs
  geometry_msgs
  image_geometry
  orca_msgs
  rclcpp
  tf2
  tf2_ros
)

ament_export_libraries(${ORCA_SHARED_LIB})

#=============
# Test node
#=============

add_executable(
  test_node
  src/test/test_node.cpp
)

ament_target_dependencies(
  test_node
  fiducial_vlam_msgs
  geometry_msgs
  image_geometry
  orca_msgs
  rclcpp
)

#=============
# Test
#=============

add_executable(
  test_main
  src/test/test_main.cpp
  src/test/test_mw_roundtrip.cpp
  src/test/test_mw_use_cases.cpp
  src/mw.cpp
  src/util.cpp
)

ament_target_dependencies(
  test_main
  astar
  fiducial_vlam_msgs
  image_geometry
  orca_msgs
  rclcpp
  sensor_msgs
)

#=============
# Install
#=============

# Install C++ targets
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

# Export include directory
ament_export_include_directories(include)

# Install include directory
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install library
install(
  TARGETS ${ORCA_SHARED_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()