#!/bin/bash -x

# Create ROS workspace
mkdir -p /ros2_ws/src
cd /ros2_ws/src

# Get orca2 source
git clone https://github.com/clydemcqueen/orca2.git

# Don't build orca_driver
touch orca2/orca_driver/COLCON_IGNORE

# Required projects
git clone https://github.com/clydemcqueen/astar.git
git clone https://github.com/clydemcqueen/sim_fiducial.git
git clone https://github.com/clydemcqueen/ukf.git
git clone https://github.com/ptrmu/fiducial_vlam_sam.git
git clone https://github.com/ptrmu/ros2_shared.git

# Forked gazebo_ros_pkgs, supports wall clock time
git clone https://github.com/clydemcqueen/gazebo_ros_pkgs.git -b foxy

# Copy setup.sh into the workspace for convenience
cp /scripts/setup.sh /ros2_ws