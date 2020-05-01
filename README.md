# Orca2 #

Orca2 is a [ROS2](http://www.ros.org/) AUV (Autonomous Underwater Vehicle) based on the
[BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).

## Simulation

Orca2 runs on Ubuntu 18.04 in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Installation instructions:

* Install ROS2 Eloquent
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/).
Use the `ros-eloquent-desktop` option.

* Install Colcon (the build tool for ROS2)
[using these instructions](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/).

* Install Gazebo v9:
~~~
sudo apt install gazebo9 libgazebo9 libgazebo9-dev
~~~

* Install these additional ROS2 packages:
~~~
sudo apt install ros-eloquent-cv-bridge ros-eloquent-camera-calibration-parsers ros-eloquent-camera-info-manager ros-eloquent-gazebo-ros-pkgs ros-eloquent-xacro
~~~

* Use your favorite Python package manager to install these Python packages:
~~~
pip3 install numpy transformations
~~~

* Build Orca2 (but not the orca_driver hardware interface):
~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/orca2.git
touch orca2/orca_driver/COLCON_IGNORE
git clone https://github.com/clydemcqueen/ukf.git
git clone https://github.com/ptrmu/fiducial_vlam.git
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/clydemcqueen/sim_fiducial.git
git clone https://github.com/clydemcqueen/astar.git
cd ~/ros2/orca2_ws
source /opt/ros/eloquent/setup.bash
colcon build
~~~

* Run the simulation:
~~~
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
source install/local_setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
source /usr/share/gazebo/setup.sh
ros2 launch orca_gazebo sim_launch.py
~~~

A few of the XBox controls:
* window button: disarm (stop all thrusters, disable all buttons except "arm")
* menu button: arm (enable all buttons, go to ROV mode). Use the joystick to move around in ROV mode.
* A: go to ROV mode
* B: go to ROV mode, and hold depth using the barometer
* X: autonomously keep station at current pose
* Y: autonomously visit all markers in a random pattern

You can also use the ros2 action CLI to start missions. E.g., to visit all markers in sequence:
~~~
ros2 action send_goal /mission orca_msgs/action/Mission '{}'
~~~

To visit markers 1, 2 and 3 in random order:
~~~
ros2 action send_goal /mission orca_msgs/action/Mission '{marker_ids: [1, 2, 3], random: true}'
~~~

To keep station at the current pose (requires a good pose):
~~~
ros2 action send_goal /mission orca_msgs/action/Mission '{pose_targets: true, keep_station: true}'
~~~

To move through 4 poses:
~~~
ros2 action send_goal /mission orca_msgs/action/Mission '{pose_targets: true, poses: [ 
{position: {x: 6.5, y: 0.5, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}},
{position: {x: 6.5, y: -0.5, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}},
{position: {x: 6.5, y: 0.5, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}},
{position: {x: 6.5, y: -0.5, z: -0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}},
]}'
~~~

See Mission.action for more details.

Orca uses camera(s) and ArUco markers to localize.
Make sure the camera has a good view of a marker before starting a mission.
If the mission fails to start there might be no marker in view, or the closest marker might be too far away.

## Packages

Orca consists of 7 packages:
* `orca_msgs` defines custom messages
* `orca_description` provides the URDF file and an URDF parser
* `orca_driver` consists of 1 node:
  * `driver_node` must run on the sub hardware. It subscribes to `Control` messages and 
  publishes `Barometer` and `Driver` messages. This node can't be tested in simulation,
  so it is kept small as possible, with the least number of dependencies.
  * Depends on `orca_msgs`

* `orca_shared` contains various shared utilities.
  * Depends on `orca_msgs`

* `orca_filter` consists of 3 nodes:
  * `depth_node` subscribes to `Barometer` messages and publishes `Depth` messages
  * `fp_node` subscribes to observations and poses from `fiducial_vlam` and publishes `FiducialPoseStamped`
  * `filter_node` fuses `FiducialPoseStamped` and `Depth` measurements and publishes filtered `FiducialPoseStamped`  
  * Depends on `orca_description`
  * Depends on `orca_msgs`
  * Depends on `orca_shared`

* `orca_base` is the largest package, and consists of 2 primary nodes:
  * `auv_node` provides the `Mission` action server and runs autonomous missions, consuming `FiducialPoseStamped`
   messages and publishing `Control` messages
  * `rov_node` subscribes to `Joy` messages and publishes `Control` messages.
It also calls the `Mission` action server to start missions
  * There are several Python nodes that use `matplotlib` to graph various messages
  * Depends on `orca_description`
  * Depends on `orca_msgs`
  * Depends on `orca_shared`

* `orca_gazebo` provides a simulation environment, with plugins for thrusters, buoyancy and drag
  * Depends on `orca_description`
  * Depends on `orca_msgs`
  * Depends on `orca_shared`

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
