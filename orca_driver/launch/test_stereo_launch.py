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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Useful for testing the stereo image processing pipeline
# Uses ROS2 components and IPC
# Requires gscam2 driver
# Not used in ft3

# Setup:
# 2x RPi camera module 2 with a wide angle lens sold by BlueRobotics
# 2x RPi Zero running start_lcam.sh and start_rcam.sh

# GSCam config:
wa1_cfg = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay'\
          ' ! avdec_h264 ! videoconvert'
wa2_cfg = 'udpsrc port=5602 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay'\
          ' ! avdec_h264 ! videoconvert'


def generate_launch_description():
    output = 'screen'

    orca_description_path = get_package_share_directory('orca_description')
    orca_driver_path = get_package_share_directory('orca_driver')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    map_path = os.path.join(orca_driver_path, 'maps', 'simple_map.yaml')

    left_camera_info = 'file://' + get_package_share_directory(
        'orca_driver') + '/cfg/wa1_dry_800x600.yaml'
    right_camera_info = 'file://' + get_package_share_directory(
        'orca_driver') + '/cfg/wa2_dry_800x600.yaml'

    # Must match the URDF file
    # base_link_frame = 'base_link'
    left_camera_name = 'left_camera'
    left_camera_name_full = '/' + left_camera_name
    left_camera_frame = 'left_camera_frame'
    right_camera_name = 'right_camera'
    right_camera_name_full = '/' + right_camera_name
    right_camera_frame = 'right_camera_frame'

    nodes = [
        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output=output,
             arguments=[urdf_path]),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_main', output=output,
             node_name='vmap', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length for new maps
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
             }]),

        # Filter
        Node(package='orca_filter', node_executable='filter_node', output=output,
             node_name='filter', parameters=[{
                'param_fluid_density': 997.0,
                'baro_init': 0,  # Init in-air
                'predict_accel': False,
                'predict_accel_control': False,
                'predict_accel_drag': False,
                'predict_accel_buoyancy': False,
                'filter_baro': False,
                'filter_fcam': False,
                'filter_lcam': True,
                'filter_rcam': True,
                'urdf_file': urdf_path,
                'urdf_barometer_joint': 'baro_joint',
                'urdf_left_camera_joint': 'left_camera_frame_joint',
                'urdf_right_camera_joint': 'right_camera_frame_joint',
             }], remappings=[
                ('lcam_f_map', '/' + left_camera_name + '/camera_pose'),
                ('rcam_f_map', '/' + right_camera_name + '/camera_pose'),
             ]),
    ]

    # 1: compose nodes at launch
    # 2: link nodes manually
    composition_method = 1

    left_vloc_params = {
        'publish_tfs': 0,
        'publish_tfs_per_marker': 0,
        'sub_camera_info_best_effort_not_reliable': 1,
        'publish_camera_pose': 1,
        'publish_base_pose': 0,
        'publish_camera_odom': 1,
        'publish_base_odom': 0,
        'stamp_msgs_with_current_time': 0,
        'camera_frame_id': left_camera_frame,
    }

    right_vloc_params = {
        'publish_tfs': 0,
        'publish_tfs_per_marker': 0,
        'sub_camera_info_best_effort_not_reliable': 1,
        'publish_camera_pose': 1,
        'publish_base_pose': 0,
        'publish_camera_odom': 1,
        'publish_base_odom': 0,
        'stamp_msgs_with_current_time': 0,
        'camera_frame_id': right_camera_frame,
    }

    left_gscam_params = {
        'gscam_config': wa1_cfg,
        'camera_name': left_camera_name,
        'camera_info_url': left_camera_info,
        'frame_id': left_camera_frame
    }

    right_gscam_params = {
        'gscam_config': wa2_cfg,
        'camera_name': right_camera_name,
        'camera_info_url': right_camera_info,
        'frame_id': right_camera_frame
    }

    if composition_method == 1:

        nodes.append(
            ComposableNodeContainer(
                package='rclcpp_components', node_executable='component_container', output=output,
                node_name='composite', node_namespace=left_camera_name_full,
                composable_node_descriptions=[
                    ComposableNode(package='fiducial_vlam', node_plugin='fiducial_vlam::VlocNode',
                                   node_name='vloc',
                                   node_namespace=left_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[left_vloc_params]),
                    ComposableNode(package='gscam', node_plugin='gscam::GSCamNode',
                                   node_name='gscam',
                                   node_namespace=left_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[left_gscam_params]),
                ]))

        nodes.append(
            ComposableNodeContainer(
                package='rclcpp_components', node_executable='component_container', output=output,
                node_name='composite', node_namespace=right_camera_name_full,
                composable_node_descriptions=[
                    ComposableNode(package='fiducial_vlam', node_plugin='fiducial_vlam::VlocNode',
                                   node_name='vloc',
                                   node_namespace=right_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[right_vloc_params]),
                    ComposableNode(package='gscam', node_plugin='gscam::GSCamNode',
                                   node_name='gscam',
                                   node_namespace=right_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[right_gscam_params]),
                ]))

    elif composition_method == 2:

        # Brittle: we're counting on the fact that the parameter names don't collide

        nodes.append(
            Node(package='orca_driver', node_executable='gscam_vloc_main', output=output,
                 node_namespace=left_camera_name_full,
                 parameters=[left_vloc_params, left_gscam_params]))

        nodes.append(
            Node(package='orca_driver', node_executable='gscam_vloc_main', output=output,
                 node_namespace=right_camera_name_full,
                 parameters=[right_vloc_params, right_gscam_params]))

    return LaunchDescription(nodes)
