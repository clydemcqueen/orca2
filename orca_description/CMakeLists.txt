cmake_minimum_required(VERSION 3.5)
project(orca_description)

find_package(ament_cmake REQUIRED)
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
# Install
#=============

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf"
  DESTINATION share/${PROJECT_NAME}
)


ament_package()