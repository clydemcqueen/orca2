# Orca2 #

Orca2 is a [ROS2](http://www.ros.org/) AUV (Autonomous Underwater Vehicle) based on the
[BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).

## Simulation

Orca2 runs in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Use the instructions below to install ROS, Gazebo and Orca2 on your desktop or laptop.

[Install ROS2 Crystal](https://index.ros.org/doc/ros2/Installation/)
with the `ros-crystal-desktop` option.
If you install binaries, be sure to also install the development tools and ROS tools from the
[source installation instructions](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/).

Install Gazebo v9:
~~~
sudo apt install gazebo9 libgazebo9 libgazebo9-dev
~~~

Install these ROS packages:
~~~
sudo apt install ros-crystal-cv-bridge ros-crystal-camera-calibration-parsers ros-crystal-camera-info-manager ros-crystal-gazebo-ros-pkgs
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
git clone https://github.com/clydemcqueen/odom_filter.git
source /opt/ros/crystal/setup.bash
colcon build
~~~

Run the simulation:
~~~
cd ~/ros2/orca_ws
source /opt/ros/crystal/setup.bash
source install/local_setup.bash
export GAZEBO_MODEL_PATH=install/orca_gazebo/share/orca_gazebo/models
ros2 launch orca_gazebo sim_launch.py
~~~

If you run into a dynamic linking problem ("libCameraPlugin.so: cannot open shared object file")
try [this workaround](https://answers.ros.org/question/313761/camera-plugin-failed-to-load-on-crystal/):
~~~
cd ~/ros2/orca_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b crystal
cd ~/ros2/orca_ws
colcon build
source install/local_setup.bash
export GAZEBO_PLUGIN_PATH=~/ros2/orca_ws/install/gazebo_plugins/lib
cp /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/* ~/ros2/orca_ws/install/gazebo_plugins/lib
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
* TODO down-facing camera
