# Orca2 #

Orca2 is a [ROS2](http://www.ros.org/) AUV (Autonomous Underwater Vehicle) based on the
[BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).

## Simulation

Orca2 runs in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Use the instructions below to install ROS, Gazebo and Orca2 on your desktop or laptop.

[Install ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/)
with the `ros-eloquent-desktop` option.
If you install binaries, be sure to also install the development tools and ROS tools from the
[source installation instructions](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/).

Install Gazebo v9:
~~~
sudo apt install gazebo9 libgazebo9 libgazebo9-dev
~~~

Install these ROS packages:
~~~
sudo apt install ros-eloquent-cv-bridge ros-eloquent-camera-calibration-parsers ros-eloquent-camera-info-manager ros-eloquent-gazebo-ros-pkgs ros-eloquent-xacro
~~~

Install MRAA header (required for the hardware interface, not required to run the simulation):

~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install libmraa2 libmraa-dev libmraa-java python-mraa python3-mraa node-mraa mraa-tools
~~~

Use your favorite Python package manager to install these Python packages:
~~~
pip3 install numpy transformations
~~~

Build Orca2:
~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/orca2.git
git clone https://github.com/clydemcqueen/ukf.git
git clone https://github.com/ptrmu/fiducial_vlam.git
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/clydemcqueen/sim_fiducial.git
git clone https://github.com/clydemcqueen/BlueRobotics_MS5837_Library.git -b mraa_ros2
cd ~/ros2/orca2_ws
source /opt/ros/eloquent/setup.bash
colcon build
~~~

Run the simulation:
~~~
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
source install/local_setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
source /usr/share/gazebo/setup.sh
ros2 launch orca_gazebo sim_launch.py
~~~

## Hardware modifications

This is rough sketch of the hardware modifications I made to my 2017 BlueROV2. YMMV.

* Remove the Pixhawk and its UBEC
* Remove the Raspberry Pi and its UBEC
* Replace the Raspberry Pi camera with the BlueRobotics USB low-light camera
* Replace the R2 ESCs with BlueRobotics R3 ESCs
* Replace the 8-conductor tether with the BlueRobotics slim tether
* Install the UP Board and a 4A UBEC
* Install the Maestro and connect to UP via USB; set the jumper to isolate the Maestro power rail from USB-provide power
* Connect the Bar30 to the UP I2C and 3.3V power pins
* Connect the ESCs to the Maestro; cut the power wire on all but one ESC
* Connect the camera tilt servo to the Maestro
* Connect the lights signal wire to the Maestro; provide power and ground directly from the battery
* Connect the leak detector to a Maestro digital input
* Build a voltage divider to provide a voltage signal from the battery (0-17V) to a Maestro analog input (0-5V)
* Connect the camera to the UP Board
