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


# Launch sub nodes


def generate_launch_description():
    # Camera name must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'
    fps = 30
    size = '800x600'

    orca_driver_path = get_package_share_directory('orca_driver')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_dry_' + size + '.ini')

    return LaunchDescription([
        Node(package='orca_driver', node_executable='barometer_node', output='screen',
             node_name='barometer_node'),

        Node(package='orca_driver', node_executable='driver_node', output='screen',
             node_name='driver_node',
             parameters=[{
                 'thruster_4_reverse': True,  # Thruster 4 on my BlueROV2 is reversed
             }]),

        Node(package='h264_image_transport', node_executable='h264_cam_node', output='screen',
             node_name='h264_cam_node', node_namespace=camera_name, parameters=[{
                'input_fn': '/dev/video1',
                'fps': fps,
                'size': size,
                'frame_id': camera_frame,
                'camera_info_path': camera_info_path,
             }]),
    ])
