#!/usr/bin/env bash

# Start orca ROS driver

# Service is disabled for ft3... launch sub_launch.py manually

# To install:           sudo cp ~/ros2/orca_ws/src/orca2/orca_driver/scripts/orca_driver.service /lib/systemd/system
#                       cp ~/ros2/orca_ws/src/orca2/orca_driver/scripts/setup.bash ~/ros2/orca_ws
# To start:             sudo systemctl start orca_driver.service
# To stop:              sudo systemctl stop orca_driver.service
# To start on boot:     sudo systemctl enable orca_driver.service
# To not start on boot: sudo systemctl disable orca_driver.service

screen -dmS orca_driver bash -c "cd ~/ros2/orca_ws; . setup.bash; ros2 launch orca_driver sub_launch.py; exec bash"
