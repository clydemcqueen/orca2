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


# Nodes for remote control and monitoring


def generate_launch_description():
    # Must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'

    all_entities = [
        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # Decode h264 stream
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
    ]

    return LaunchDescription(all_entities)
