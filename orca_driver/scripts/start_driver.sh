#!/usr/bin/env bash

# Start orca ROS driver

# To install:           sudo cp orca_driver.service /lib/systemd/system
# To start:             sudo systemctl start orca_driver.service
# To stop:              sudo systemctl stop orca_driver.service
# To start on boot:     sudo systemctl enable orca_driver.service
# To not start on boot: sudo systemctl disable orca_driver.service

screen -dmS orca bash -c "cd ~/ros2/orca_ws; . setup.bash; ros2 launch orca_driver sub_launch.py; exec bash"
