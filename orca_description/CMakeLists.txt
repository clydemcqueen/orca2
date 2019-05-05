cmake_minimum_required(VERSION 3.5)
project(orca_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/orca_description)

ament_package()