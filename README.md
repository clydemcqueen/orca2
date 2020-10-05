# Orca2 #

Orca2 is a [ROS2](http://www.ros.org/) AUV (Autonomous Underwater Vehicle) based on the
[BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).

## Simulation

Orca2 runs on Ubuntu 18.04 in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Installation instructions:

* Install ROS2 Eloquent
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/).
Use the `ros-eloquent-desktop` option.

* Install ROS2 development tools
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/).
Stop after installing the development tools (before "Get ROS 2 code").

* Initialize rosdep:
~~~
sudo rosdep init
rosdep update
~~~

* Install Gazebo v9:
~~~
sudo apt install gazebo9 libgazebo9 libgazebo9-dev
~~~

* Use your favorite Python package manager to install these Python packages:
~~~
pip3 install numpy transformations
~~~

* Orca2 uses [fiducial_vlam_sam](https://github.com/ptrmu/fiducial_vlam_sam),
which in turn uses [GTSAM](https://github.com/borglab/gtsam).
You will need to install GTSAM.
Alternatively you can swap [fiducial_vlam](https://github.com/ptrmu/fiducial_vlam) for fiducial_vlam_sam,
but the parameters are different, so the launch files will need to be modified.

* Build Orca2 (orca_driver is not required for the simulation):
~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/orca2.git
touch orca2/orca_driver/COLCON_IGNORE
git clone https://github.com/clydemcqueen/ukf.git
git clone https://github.com/ptrmu/fiducial_vlam_sam.git
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/clydemcqueen/sim_fiducial.git
git clone https://github.com/clydemcqueen/astar.git
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
# Install system dependencies:
rosdep install --from-paths . --ignore-src
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

* `orca_driver` runs on the sub hardware, and consists of 2 nodes:
  * `barometer_node` reads the Bar30 and publishes `Barometer` messages
  * `driver_node` subscribes to `Control` messages and publishes `Driver` messages
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

## Status and Future Work

I built the sub and ran it through a series of tests with mixed results.
There's still a lot of work to do in vehicle dynamics (understanding drag, tuning PID controllers).

I'm currently evaluating [ROS Nav2](https://navigation.ros.org/). One idea is to use Nav2
and add plugins for specific Orca2 control strategies.

Possible future sensors:
* down-facing camera(s), useful for measuring distance to the seafloor and for visual odometry
* forward-facing and/or down-facing lasers for measuring distance
* sonar for obstacle avoidance
