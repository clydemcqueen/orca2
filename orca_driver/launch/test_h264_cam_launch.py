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
    camera_name = 'forward_camera'
    # camera_frame = 'forward_camera_frame'
    fps = 30
    size = '1920x1080'

    orca_driver_path = get_package_share_directory('orca_driver')
    # params_path = os.path.join(orca_driver_path, 'launch', 'ft3_params.yaml')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_dry_' + size + '.ini')
    # map_path = os.path.join(orca_driver_path, 'maps', 'ft3_map.yaml')

    return LaunchDescription([
        # Forward camera h264
        # Will publish on image_raw/h264
        Node(package='h264_image_transport', node_executable='h264_cam_node', output='screen',
             node_name='h264_cam_node', node_namespace=camera_name, parameters=[{
                'input_fn': '/dev/video2',
                'fps': fps,
                'size': size,
                'frame_id': 'camera_frame',
                'camera_info_path': camera_info_path,
             }]),

        # Load and publish a known map
        # Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
        #      node_name='vmap_node', parameters=[{
        #         'publish_tfs': 1,  # Publish marker /tf
        #         'marker_length': 0.1778,  # Marker length for new maps
        #         'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
        #         'make_not_use_map': 0  # Don't modify the map
        #     }]),

        # Pick one transport and republish for vloc
        Node(package='image_transport', node_executable='republish', output='screen',
             node_name='republish_node', node_namespace=camera_name, arguments=[
                'h264',  # Input
                'raw'  # Output
             ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('in/h264', 'image_raw/h264'),
                ('out', 'repub_raw'),
                ('out/compressed', 'repub_raw/compressed'),
                ('out/theora', 'repub_raw/theora'),
                ('out/theora', 'repub_raw/h264'),
             ]),

        # Localize against the map
        # Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
        #      node_name='vloc_node', node_namespace=camera_name, parameters=[
        #         params_path, {
        #             'camera_frame_id': camera_frame,
        #         }], remappings=[
        #         ('image_raw', 'repub_raw'),
        #     ]),

        # Measure lag -- subscribe to repub_raw
        # Node(package='pipe_perf', node_executable='image_sub_node', output='screen',
        #      node_name='image_sub_node', node_namespace=camera_name, remappings=[
        #         ('image_raw', 'repub_raw'),
        #     ]),

        # Measure lag -- subscribe to image_raw/h264
        Node(package='pipe_perf', node_executable='image_transport_sub_node', output='screen',
             node_name='image_transport_sub_node', node_namespace=camera_name),
    ])
