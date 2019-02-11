Simulation environment for [Orca2](https://github.com/clydemcqueen/orca2).

Running into a dynamic linking problem... not sure what's going on.

Workaround:
* add ros2 branch of gazebo_ros_pkgs to workspace, and build it
* export GAZEBO_PLUGIN_PATH=~/ros2/orca_ws/install/gazebo_plugins/lib
* cp /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/* ~/ros2/orca_ws/install/gazebo_plugins/lib

Test the workaround:
* gazebo --verbose /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_camera_demo.world
