#!/usr/bin/env bash

# Start orca ROS driver

# To install:           sudo cp orca_driver.service /lib/systemd/system
# To start:             sudo systemctl start orca_driver.service
# To stop:              sudo systemctl stop orca_driver.service
# To start on boot:     sudo systemctl enable orca_driver.service
# To not start on boot: sudo systemctl disable orca_driver.service

# TODO add ~/ms5837-python to PYTHONPATH
# TODO scrub ref to /home/clyde

bash -c "source /opt/ros/crystal/setup.bash && source /home/clyde/ros2/orca_ws/install/local_setup.bash && stdbuf -o L ros2 launch orca_driver gscam_launch.py"
