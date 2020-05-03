cmake_minimum_required(VERSION 3.5)
project(orca_description)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

#=============
# Run Xacro to turn orca.urdf.xacro into orca.urdf
# Source file is in ${CMAKE_CURRENT_SOURCE_DIR}/xacro/orca.urdf.xacro
# Generated file is in ${CMAKE_CURRENT_BINARY_DIR}/urdf
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf")

set(SUFFIX "")
set(TOPIC_NS "solo")

set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/urdf/orca.urdf")

add_custom_command(
  OUTPUT ${URDF_FILE}
  COMMAND ${PYTHON_EXECUTABLE} "/opt/ros/eloquent/bin/xacro"
  "${CMAKE_CURRENT_SOURCE_DIR}/xacro/orca.urdf.xacro" ">" "${URDF_FILE}"
  DEPENDS xacro/orca.urdf.xacro
  COMMENT "Generate ${URDF_FILE}"
  VERBATIM
)

add_custom_target(generate_urdf ALL DEPENDS ${URDF_FILE})

#=============
# Library
#=============

# Local includes
include_directories(
  include
)

set(ORCA_DESCRIPTION_LIB "orca_description")

add_library(
  ${ORCA_DESCRIPTION_LIB} SHARED
  src/parser.cpp
)

ament_target_dependencies(
  ${ORCA_DESCRIPTION_LIB}
  tf2
  urdf
)

#=============
# Utilities for testing pipeline performance
#=============

add_executable(
  time_send_node
  src/time_send.cpp
)

ament_target_dependencies(
  time_send_node
  std_msgs
  rclcpp
)

add_executable(
  time_receive_node
  src/time_receive.cpp
)

ament_target_dependencies(
  time_receive_node
  std_msgs
  rclcpp
)

#=============
# Install
#=============

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf"
  DESTINATION share/${PROJECT_NAME}
)

# Export include directory
ament_export_include_directories(include)

# Install include directory
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Export library
ament_export_libraries(${ORCA_DESCRIPTION_LIB})

# Install library
install(
  TARGETS ${ORCA_DESCRIPTION_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install nodes
install(
  TARGETS time_send_node time_receive_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()