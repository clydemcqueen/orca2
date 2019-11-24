Robot description files for [Orca2](https://github.com/clydemcqueen/orca2).

Colcon will run [Xacro](https://index.ros.org/r/xacro/github-ros-xacro/) to create `orca.urdf` from `orca.urdf.xacro`.
To convert manually:

~~~
# Install Xacro
sudo apt install ros-eloquent-xacro

# Source Eloquent setup.bash
source /opt/ros/eloquent/setup.bash

# Run in orca_description/urdf directory
cd ~/orca_ws/src/orca2/orca_description/urdf
python3 /opt/ros/melodic/bin/xacro orca.urdf.xacro > orca.urdf
~~~