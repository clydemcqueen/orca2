# Hardware Interface

## Hardware Build

The current hardware build is called FT3 (Field Test #3).
This configuration includes a tether, with most nodes running on a desktop.

I made the following standard modifications to my 2017 BlueRobotics BlueROV2:

* Replaced the Raspberry Pi camera with the BlueRobotics USB low-light camera
* Replaced the R2 ESCs with BlueRobotics R3 ESCs
* Replaced the 8-conductor tether with the 2-conductor slim tether

I made the following custom modifications -- YMMV:

* Replaced the Pixhawk with a Pololu Maestro 18
* Built a voltage divider to provide a voltage signal from the battery (0-17V) to a Maestro analog input (0-5V)
* There is no current sensor
* There is no IMU

## Raspberry Pi Software Installation

The Raspberry Pi is now the primary controller.
Below I've outlined rough instructions... you'll need to dive into the system-specific instructions for details.

### Install Ubuntu 18.04.4 LTS Server

Install Ubuntu 18.04.4 LTS Server for ARM64
[using these instructions](https://wiki.ubuntu.com/ARM/RaspberryPi).

### Install ffmpeg

Check the requirements for [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport)
and install any missing ffmpeg libraries.

### Install ROS2 Eloquent

Install ROS2 Eloquent
[using these instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/).
Use the `ros-eloquent-ros-base` option to avoid installing the GUI tools.

Install Colcon (the build tool for ROS2)
[using these instructions](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/).

Install these additional packages:
~~~
sudo apt-get install ros-eloquent-image-transport ros-eloquent-image-transport-plugins ros-eloquent-camera-calibration-parsers
~~~

### Install MRAA

MRAA is a standard hardware interface for small boards.

~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install libmraa2 libmraa-dev libmraa-java python3-mraa node-mraa mraa-tools
~~~

### Install Orca

~~~
mkdir -p ~/ros2/orca_ws/src
cd ~/ros2/orca_ws/src
git clone https://github.com/clydemcqueen/BlueRobotics_MS5837_Library.git -b mraa_ros2
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/ptrmu/fiducial_vlam.git
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
colcon build
source install/local_setup.bash
~~~

## Manual Launch

On the Raspberry Pi:

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

## Launch on Boot

To configure the Raspberry Pi to launch on boot:

~~~
sudo cp ~/ros2/orca_ws/src/orca2/orca_driver/scripts/orca_driver.service /lib/systemd/system
sudo systemctl enable orca_driver.service
~~~

## Troubleshooting

The compiler (`cc1plus`) sometimes dies during build, try restarting `colcon build`.
If that fails, try:
~~~
export MAKEFLAGS='-j 1'
colcon build
~~~

Test the i2c bus using `i2cdetect 1`.

`raspi-config` isn't installed as part of Ubuntu 18.04.4, and the PPA I found has a few bugs.
I hand-edited the boot sector config files on the SD card and added `dtparam=i2c_arm=on`.

Make sure that `$USER` is in the dialout, video and i2c groups.
Remember to log out and log back in after changing groups.