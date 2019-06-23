Robot description files for [Orca2](https://github.com/clydemcqueen/orca2).

Xacro hasn't been ported to Python3 yet.
Use the version from ROS Melodic instead, and be careful about running in Python2.
~~~
# Run in orca_description/urdf directory
# Must source /opt/ros/melodic/setup.bash first
python2 /opt/ros/melodic/bin/xacro orca.urdf.xacro > orca.urdf
python2 /opt/ros/melodic/bin/xacro pt2.urdf.xacro > pt2.urdf
~~~