#!/usr/bin/env python3

# Copyright (c) 2020, Clyde McQueen.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Useful for testing a single camera pipeline


def generate_launch_description():
    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_driver_path = get_package_share_directory('orca_driver')
    # params_path = os.path.join(orca_driver_path, 'launch', 'ft3_params.yaml')
    camera_info_url = 'file://' + os.path.join(orca_driver_path, 'cfg', 'brusb_dry_1920x1280.ini')
    # map_path = os.path.join(orca_driver_path, 'maps', 'ft3_map.yaml')

    # v4l camera => ROS images
    # cfg_v4l_ros = 'v4l2src device=/dev/video2 do-timestamp=true ! queue !' \
    #               ' video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse !' \
    #               ' avdec_h264 ! videoconvert'

    # v4l camera => mp4 fil and ROS images
    # cfg_v4l_rec_and_ros = 'v4l2src device=/dev/video2 do-timestamp=true ! queue !' \
    #                       ' video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse !' \
    #                       ' tee name=fork ! queue ! mp4mux !' \
    #                       ' filesink location=save.mp4 fork. !' \
    #                       ' avdec_h264 ! videoconvert'

    # udp stream => ROS images
    # cfg_rcv_ros = 'udpsrc port=5600 ! queue ! application/x-rtp ! rtph264depay ! h264parse !' \
    #               ' avdec_h264 ! videoconvert'

    # udp stream => ROS images, pipeline from qgroundcontrol with rtpjitterbuffer
    # Adds some latency but improves image stability
    # https://github.com/mavlink/qgroundcontrol/blob/8512a64170dafe9f8dfd49762cd9dbf95064ed00/src/VideoReceiver/README.md
    # https://github.com/mavlink/qgroundcontrol/blob/8512a64170dafe9f8dfd49762cd9dbf95064ed00/src/VideoReceiver/GstVideoReceiver.cc
    cfg_rcv_qgc_ros = 'udpsrc port=5600 ! queue !' \
                      ' application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !' \
                      ' rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'

    # v4l camera => udp stream and ROS images
    # cfg_v4l_snd_and_ros = 'v4l2src device=/dev/video2 do-timestamp=true ! queue !' \
    #                       ' video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse !' \
    #                       ' tee name=fork ! queue ! rtph264pay config-interval=10 !' \
    #                       ' udpsink host=127.0.0.1 port=5600 fork. ! queue ! avdec_h264 !' \
    #                       ' videoconvert'

    # v4l camera => compressed ROS images
    # cfg_v4l_jpeg_ros = 'v4l2src device=/dev/video2 do-timestamp=true ! queue !' \
    #                    ' video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse !' \
    #                    ' avdec_h264 ! jpegenc'

    # tcp stream => ROS images
    # cfg_rcv_tcp_ros = ' tcpclientsrc port=5601 host=192.168.86.101 !' \
    #                   ' application/x-rtp-stream,encoding-name=H264 ! rtpstreamdepay !' \
    #                   ' application/x-rtp ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'

    return LaunchDescription([
        # Forward camera
        Node(package='gscam', node_executable='gscam_main', output='screen',
             node_name='gscam_node', node_namespace=camera_name, parameters=[{
                'gscam_config': cfg_rcv_qgc_ros,
                'use_gst_timestamps': False,  # TODO bug -- this isn't working
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': camera_info_url,
                'camera_name': camera_name,
                'frame_id': camera_frame,
             }]),

        # Load and publish a known map
        # Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
        #      node_name='vmap_node', parameters=[{
        #         'publish_tfs': 1,  # Publish marker /tf
        #         'marker_length': 0.1778,  # Marker length for new maps
        #         'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
        #         'make_not_use_map': 0  # Don't modify the map
        #     }]),

        # Localize against the map
        # Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
        #      node_name='vloc_node', node_namespace=camera_name, parameters=[
        #         params_path, {
        #             'camera_frame_id': camera_frame,
        #         }]),

        # Measure lag
        # Node(package='gscam', node_executable='subscriber_main', output='screen',
        #      node_name='subscriber_raw', node_namespace=camera_name),
        # Node(package='gscam', node_executable='subscriber_main', output='screen',
        #      node_name='subscriber_marked', node_namespace=camera_name, remappings=[
        #         ('image_raw', 'image_marked'),
        #     ]),
        Node(package='pipe_perf', node_executable='image_sub_node', output='screen',
             node_name='image_sub_node', node_namespace=camera_name)

    ])
