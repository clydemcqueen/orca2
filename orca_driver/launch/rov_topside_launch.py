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


# Launch topside nodes -- ROV only


def generate_launch_description():
    # Each ballast weight weighs 0.19kg
    mass = 11.1
    volume = 0.01148
    fluid_density = 997.0

    orca_description_path = get_package_share_directory('orca_description')
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    all_entities = [
        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='log',
             arguments=[urdf_path]),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
             }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[{
                'mass': mass,
                'volume': volume,
                'fluid_density': fluid_density,
                'planner_target_z': -0.2,
            }]),
    ]

    return LaunchDescription(all_entities)
