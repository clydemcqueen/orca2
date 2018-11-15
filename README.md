# Orca2 #

Orca2 is a [ROS2](https://index.ros.org/doc/ros2/) port of [Orca](https://github.com/clydemcqueen/orca).

## Status

The overall goal is to port Orca to ROS2. It's possible that some components won't be ported. 

* `orca_msgs` ported
* `orca_description` ported, missing features
* `orca_driver` blocked on several packages
* `orca_base` ported, missing features
* `orca_topside` not ported
* `orca_gazebo` ported
* `orca_vision` blocked on several packages

## Requirements

* Ubuntu 18.04
* ROS2 Bouncy
* gazebo_ros_pkgs

## Building

TODO

## Running

* `gazebo --verbose ~/orca_ws/src/orca/orca_gazebo/worlds/orca.world`
* `ros2 launch orca_description description_launch.py`
* `ros2 launch orca_base base_launch.py`
* `rviz2`