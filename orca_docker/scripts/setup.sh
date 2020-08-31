#!/bin/bash -x

# Get ready to build & run

# Clear the decks
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH

# ROS
source /opt/ros/eloquent/setup.bash
source install/local_setup.bash

# Log behavior, should not be required in Foxy
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

# OpenCV and GTSAM
export CMAKE_PREFIX_PATH=/lib/gtsam/install/lib/cmake/GTSAM:/lib/opencv/install/lib/cmake/opencv4:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/lib/gtsam/install/lib/:/lib/opencv/install/lib/:$LD_LIBRARY_PATH

# Gazebo
export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
source /usr/share/gazebo/setup.sh
