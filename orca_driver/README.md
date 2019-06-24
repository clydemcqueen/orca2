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
sudo apt install ros-dashing-camera-calibration-parsers ros-dashing-camera-info-manager

cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/gscam.git -b ros2
cd ~/ros2/orca2_ws
colcon build
~~~

For the Bar30:
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