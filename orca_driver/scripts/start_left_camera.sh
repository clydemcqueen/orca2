#!/usr/bin/env bash

# TODO not used in pt2

# Start video stream for left down-facing camera

# To install:           sudo cp orca_left_camera.service /lib/systemd/system
# To start:             sudo systemctl start orca_left_camera.service
# To stop:              sudo systemctl stop orca_left_camera.service
# To start on boot:     sudo systemctl enable orca_left_camera.service
# To not start on boot: sudo systemctl disable orca_left_camera.service

# For USB cam (high res):
# gst-launch-1.0 -v v4l2src device=/dev/video1 do-timestamp=true ! queue ! "video/x-h264, stream-format=(string)byte-stream, alignment=(string)au, width=(int)1920, height=(int)1080, pixel-aspect-ratio=(fraction)1/1, framerate=(fraction)30/1" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.86.105 port=5600

# For Raspicam (high res):
# raspivid --nopreview --mode 5 --bitrate 15000000 --intra 1 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=5600

# For Raspicam (reduced res for image processing):
raspivid --nopreview --mode 4 -w 800 -h 600 --framerate 15 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.7.1 port=5600

