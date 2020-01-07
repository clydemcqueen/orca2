Hardware interface for [Orca2](https://github.com/clydemcqueen/orca2).

TODO: split install instructions for device builds and simulation builds.
Move device build instructions here, and simulation build instructions to orca_gazebo/README.md.

Minimal ROS install:
~~~
sudo apt install ros-eloquent-ros-base ros-eloquent-cv-bridge ros-eloquent-yaml-cpp-vendor
~~~

Don't build orca_gazebo (and possibly other packages) on the sub:
~~~
touch ~/ros2/orca_ws/src/orca2/orca_gazebo/COLCON_IGNORE
touch ~/ros2/orca_ws/src/orca2/orca_base/COLCON_IGNORE
touch ~/ros2/orca_ws/src/orca2/orca_description/COLCON_IGNORE
touch ~/ros2/orca_ws/src/orca2/orca_filter/COLCON_IGNORE
touch ~/ros2/orca_ws/src/orca2/orca_shared/COLCON_IGNORE
~~~

For the Raspberry Pi camera:
~~~
cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/gscam.git -b ros2
cd ~/ros2/orca2_ws
colcon build
~~~

For the USB camera:
~~~
sudo usermod -a -G video ${USER}
# Log out and back in again

sudo apt install ros-eloquent-camera-calibration-parsers

cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/opencv_cam.git
cd ~/ros2/orca2_ws
colcon build
~~~

For mraa on the UP Board, see https://wiki.up-community.org/MRAA/UPM:
~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install mraa-tools mraa-examples libmraa1 libmraa-dev libupm-dev libupm1 upm-examples
sudo apt-get install python-mraa python3-mraa node-mraa libmraa-java
~~~

For the Bar30 (requires MRAA):
~~~
cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/BlueRobotics_MS5837_Library.git -b mraa_ros2
cd ~/ros2/orca2_ws
colcon build
~~~

To record bags:
~~~
sudo apt install sqlite3 ros-eloquent-rosbag2* ros-eloquent-ros2bag

ros2 bag record -a  # Everything, or just some things:
ros2 bag record /rosout /barometer /battery /control /error /leak /tf /forward_camera/base_odom /filtered_path /fiducial_map /fiducial_markers /fiducial_observations /forward_camera/camera_info
~~~

To isolate ROS2 networks:
~~~
export ROS_DOMAIN_ID=42
~~~

To display timestamps on console messages:
~~~
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
~~~