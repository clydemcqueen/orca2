# Hardware Interface

## Hardware Build

The current hardware build is called FT3 (Field Test #3).
This configuration includes a tether, with most nodes running on a desktop.

I made the following standard modifications to my 2017 BlueRobotics BlueROV2:

* Replaced the Raspberry Pi camera with the BlueRobotics USB low-light camera
* Replaced the R2 ESCs with BlueRobotics R3 ESCs
* Replaced the 8-conductor tether with the 2-conductor slim tether

I made the following custom modifications -- YMMV:

* Replaced the Raspberry Pi with an [UP Board](https://up-board.org/up/specifications/)
* Replaced the Pixhawk with a [Pololu Maestro 18](https://www.pololu.com/product/1354)
* Built a voltage divider to provide a voltage signal from the battery (0-17V) to a Maestro analog input (0-5V)
* There is no current sensor
* There is no IMU

## Software Installation

Below I've outlined rough instructions to install the required software on the UP board.
You'll need to dive into the system-specific instructions for details.

### Install Ubuntu 18.04.4 LTS Server

Install Ubuntu 18.04.4 LTS Server.

### Install Chrony

Since the logic is split across 2 machines, the clocks need to be synchronized.
I've had good luck with [https://chrony.tuxfamily.org/doc/3.5/installation.html](Chrony).
Have the UP board use the desktop as a reference.

### Install ffmpeg

Check the requirements for [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport)
and install any missing ffmpeg libraries.

### Install ROS2 Eloquent

Install ROS2 Eloquent
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/).
Use the `ros-eloquent-ros-base` option to avoid installing the GUI tools.

Install ROS2 development tools
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/).
Stop after installing the development tools (before "Get ROS 2 code").

Initialize rosdep:
~~~
sudo rosdep init
rosdep update
~~~

### Install MRAA

MRAA is a standard hardware interface for small boards.

~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install libmraa2 libmraa-dev libmraa-java python3-mraa node-mraa mraa-tools
~~~

### Install Orca2 on the ROV

~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/BlueRobotics_MS5837_Library.git -b mraa_ros2
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/ptrmu/fiducial_vlam_sam.git
touch fiducial_vlam/fiducial_vlam/COLCON_IGNORE
git clone https://github.com/clydemcqueen/h264_image_transport.git
git clone https://github.com/clydemcqueen/orca2.git
touch orca2/orca_base/COLCON_IGNORE
touch orca2/orca_description/COLCON_IGNORE
touch orca2/orca_filter/COLCON_IGNORE
touch orca2/orca_gazebo/COLCON_IGNORE
touch orca2/orca_shared/COLCON_IGNORE
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
rosdep install --from-paths . --ignore-src
colcon build
source install/local_setup.bash
~~~

## Manual Launch

On the UP board:

~~~
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
source install/local_setup.bash
ros2 launch orca_driver sub_launch.py
~~~

On the desktop computer:

~~~
cd ~/ros2/orca_ws
source /opt/ros/eloquent/setup.bash
source install/local_setup.bash
ros2 launch orca_driver topside_launch.py
~~~

## Troubleshooting

The compiler (`cc1plus`) sometimes dies during build, try restarting `colcon build`.
If that fails, try:
~~~
export MAKEFLAGS='-j 1'
colcon build
~~~

Test the i2c bus using `i2cdetect 1`.

Make sure that `$USER` is in the dialout, video and i2c groups.
Remember to log out and log back in after changing groups.