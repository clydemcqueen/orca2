Hardware interface for [Orca2](https://github.com/clydemcqueen/orca2).

Additional install notes:
~~~
sudo apt install ros-crystal-camera-calibration-parsers ros-crystal-camera-info-manager

cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/gscam.git
cd gscam
git checkout ros2

cd ~/ros2/orca2_ws
colcon build --packages-skip orca_gazebo
~~~