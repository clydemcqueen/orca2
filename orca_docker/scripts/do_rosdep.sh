#!/bin/bash

cd ./ros2_ws
source /opt/ros/eloquent/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src
