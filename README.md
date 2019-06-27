# Orca2 #

Orca2 is a [ROS2](http://www.ros.org/) AUV (Autonomous Underwater Vehicle) based on the
[BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).

## Simulation

Orca2 runs in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Use the instructions below to install ROS, Gazebo and Orca2 on your desktop or laptop.

[Install ROS2 Dashing](https://index.ros.org/doc/ros2/Installation/)
with the `ros-dashing-desktop` option.
If you install binaries, be sure to also install the development tools and ROS tools from the
[source installation instructions](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/).

Install Gazebo v9:
~~~
sudo apt install gazebo9 libgazebo9 libgazebo9-dev
~~~

Install these ROS packages:
~~~
sudo apt install ros-dashing-cv-bridge ros-dashing-camera-calibration-parsers ros-dashing-camera-info-manager ros-dashing-gazebo-ros-pkgs
~~~

Use your favorite Python package manager to install these Python packages:
~~~
pip install numpy transformations
~~~

Build Orca2:
~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/orca2.git
git clone https://github.com/ptrmu/fiducial_vlam.git
git clone https://github.com/ptrmu/ros2_shared.git
source /opt/ros/dashing/setup.bash
colcon build
~~~

Run the simulation:
~~~
cd ~/ros2/orca_ws
source /opt/ros/dashing/setup.bash
source install/local_setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/orca_gazebo/share/orca_gazebo/models
source /usr/share/gazebo/setup.sh
ros2 launch orca_gazebo sim_launch.py
~~~

## Hardware modifications

This is rough sketch of the hardware modifications I made to the BlueROV2. YMMV.

* Remove the Pixhawk and its UBEC
* Remove the Raspberry Pi and its UBEC
* Install the UP Board and a 4A UBEC
* Install the Maestro and connect to UP via USB; set the jumper to isolate the Maestro power rail from USB-provide power
* Connect the Bar30 to the UP I2C and 3.3V power pins
* Connect the ESCs to the Maestro; cut the power wire on all but one ESC
* Connect the camera tilt servo to the Maestro
* Connect the lights signal wire to the Maestro; provide power and ground directly from the battery
* Connect the leak detector to a Maestro digital input
* Build a voltage divider to provide a voltage signal from the battery (0-17V) to a Maestro analog input (0-5V)
* Connect the BlueRobotics USB low-light camera to the UP Board
