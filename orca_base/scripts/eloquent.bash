# Handy script, store this in ~/ros2 and run `. ../eloquent.bash` from the workspace

# Clear the decks
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset GAZEBO_MODEL_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

# ROS distro and overlay
. /opt/ros/eloquent/setup.bash
. install/local_setup.bash

# Gazebo
export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
. /usr/share/gazebo/setup.sh

# Log behavior, should not be required in Foxy
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

# GTSAM
export CMAKE_PREFIX_PATH=~/lib/gtsam/install/lib/cmake/GTSAM:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=~/lib/gtsam/install/lib/:$LD_LIBRARY_PATH

# OpenCV 4.2
export CMAKE_PREFIX_PATH=~/opencv/install/opencv_4_2:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=~/opencv/install/opencv_4_2/lib:$LD_LIBRARY_PATH
