Hardware interface for [Orca2](https://github.com/clydemcqueen/orca2).

For computer vision:
~~~
sudo apt install ros-crystal-camera-calibration-parsers ros-crystal-camera-info-manager

cd ~/ros2/orca2_ws/src
git clone https://github.com/clydemcqueen/gscam.git -b ros2
cd ~/ros2/orca2_ws
colcon build --packages-skip orca_gazebo
~~~

For the Bar30:
~~~
pip install smbus future
cd ~
git clone https://github.com/clydemcqueen/ms5837-python -b python3
# TODO add ~/ms5837-python to PYTHONPATH
~~~
