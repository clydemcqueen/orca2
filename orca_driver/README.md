Hardware interface for [Orca2](https://github.com/clydemcqueen/orca2).

Minimal ROS install:
~~~
sudo apt install ros-dashing-ros-base ros-dashing-cv-bridge ros-dashing-yaml-cpp-vendor
~~~

Don't build orca_gazebo on the sub:
~~~
touch ~/ros2/orca_ws/src/orca2/orca_gazebo/COLCON_IGNORE
~~~

For the Raspberry Pi camera:
~~~
cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/gscam.git -b ros2
cd ~/ros2/orca2_ws
colcon build
~~~

For the Bar30 TODO:
~~~
pip install smbus future
cd ~
git clone https://github.com/clydemcqueen/ms5837-python -b python3
# TODO add ~/ms5837-python to PYTHONPATH
# TODO port ms5837-python to Python3
~~~

For mraa on the UP Board, see https://wiki.up-community.org/MRAA/UPM:
~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install mraa-tools mraa-examples libmraa1 libmraa-dev libupm-dev libupm1 upm-examples
sudo apt-get install python-mraa python3-mraa node-mraa libmraa-java
~~~

To record bags:
~~~
sudo apt install sqlite3 ros-dashing-rosbag2* ros-dashing-ros2bag
ros2 bag record /battery /control /error /leak /tf /forward_camera/base_odom /filtered_path /fiducial_map /fiducial_markers /fiducial_observations /forward_camera/camera_info
~~~

To isolate ROS2 networks:
~~~
export ROS_DOMAIN_ID=42
~~~

To display timestamps on console messages:
~~~
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
~~~