#!/bin/bash

# Run colcon
cd /ros2_ws
source /opt/ros/foxy/setup.bash
source setup.sh
colcon build
