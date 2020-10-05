# Handy script, store this in ~/ros2 and run `. ../foxy.bash` from the workspace

# Clear the decks
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset GAZEBO_MODEL_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

# ROS distro and overlay
. /opt/ros/foxy/setup.bash
. install/local_setup.bash

# Force logging to stdout, not stderr
export RCUTILS_LOGGING_USE_STDOUT=1

# Gazebo
. /usr/share/gazebo/setup.sh

# Gazebo -- Orca2
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PWD}/install/sim_fiducial/share/sim_fiducial/models

# Gazebo -- Turtlebot3
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PWD}/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models

# GTSAM
# export CMAKE_PREFIX_PATH=~/lib/gtsam/install/lib/cmake/GTSAM:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=~/lib/gtsam/install/lib/:$LD_LIBRARY_PATH

# OpenCV 4.2
# export CMAKE_PREFIX_PATH=~/opencv/install/opencv_4_2:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=~/opencv/install/opencv_4_2/lib:$LD_LIBRARY_PATH
