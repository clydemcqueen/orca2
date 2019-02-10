Simulation environment for [Orca2](https://github.com/clydemcqueen/orca2).

Running into a dynamic linking problem... not sure what's going on.
Workaround:
* add ros2 branch of gazebo_ros_pkgs to workspace, and build it
* export GAZEBO_PLUGIN_PATH=~/orca_ws/install/gazebo_plugins/lib
* cp /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libCameraPlugin.so ~/orca_ws/install/gazebo_plugins/lib
* gazebo --verbose /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_camera_demo.world

Then, running orca.world:
* export GAZEBO_MODEL_PATH=~/orca_ws/install/orca_gazebo/share/orca_gazebo/models
* gazebo --verbose ~/orca_ws/install/orca_gazebo/share/orca_gazebo/worlds/orca.world