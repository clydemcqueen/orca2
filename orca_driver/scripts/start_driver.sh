#!/usr/bin/env bash

# Start orca ROS driver

# To install:           sudo cp driver.service /lib/systemd/system
# To start:             sudo systemctl start driver.service
# To stop:              sudo systemctl stop driver.service
# To start on boot:     sudo systemctl enable driver.service
# To not start on boot: sudo systemctl disable driver.service

screen -dmS orca bash -c "cd ~/ros2/orca_ws; . setup.bash; ros2 launch orca_driver sub_launch.py; exec bash"
