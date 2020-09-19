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

"""Launch a simulation of a single marker plus a filter."""

from enum import Enum
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    sim_fiducial_path = get_package_share_directory('sim_fiducial')
    world_path = os.path.join(sim_fiducial_path, 'worlds', 'one_marker.world')
    map_path = os.path.join(sim_fiducial_path, 'worlds', 'one_marker_map.yaml')
    sdf_path = os.path.join(sim_fiducial_path, 'sdf', 'forward_camera.sdf')

    vmap_node_params = {
        # Publish marker /tf
        'psm_publish_tfs': 1,

        # Load map
        'map_load_filename': map_path,

        # Don't save the map
        'map_save_filename': '',
    }

    model_params = {
        # Match orca.urdf (slight positive buoyancy):
        'mdl_mass': 9.9,
        'mdl_volume': 0.01,
        'mdl_fluid_density': 997.0,
    }

    pose_filter_node_params = {
        'four_dof': True,  # Simplify calcs
        'predict_accel': False,  # No control input, drag or buoyancy in this sim
        'predict_accel_control': False,
        'predict_accel_drag': False,
        'predict_accel_buoyancy': False,
        'filter_baro': False,  # No depth in this sim
        'filter_fcam': True,
        'publish_tf': True,  # Publish map=>base tf for rviz
        'good_pose_dist': 5.0,  # Marker is 2-3m away, so make this pretty far
        'good_obs_dist': 10.0,

        # Process noise, similar to /depth noise
        'ukf_process_noise': 0.0004,

        # Turn outlier detection off
        'ukf_outlier_distance': -1.0,
    }
    pose_filter_node_params.update(model_params)

    all_entities = [
        # Launch Gazebo, loading the world
        ExecuteProcess(cmd=[
            # 'gazebo',
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the sdf to the simulation
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[sdf_path, '0', '0', '0', '0', '0', '0']),

        # Publish, and possibly build, a map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[vmap_node_params]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=camera_name, parameters=[{
                # Localize, don't calibrate
                'loc_calibrate_not_loocalize': 0,

                # 0: OpenCV, 1: GTSAM
                'loc_camera_sam_not_cv': 1,

                'psl_camera_frame_id': camera_frame,
                'psl_publish_tfs': 0,

                # 0: DICT_4x4_50 (default)
                # 8: DICT_6X6_250
                'loc_aruco_dictionary_id': 8,

                # Gazebo publishes camera info best-effort
                'psl_sub_camera_info_best_effort_not_reliable': 1,

                # Publish the camera pose
                'psl_publish_camera_pose': 1,

                # Don't publish anything else
                'psl_publish_base_pose': 0,
                'psl_publish_camera_odom': 0,
                'psl_publish_base_odom': 0,

                # Keep the existing timestamps
                'psl_stamp_msgs_with_current_time': 0,
            }]),

        # Combine poses and observations into fiducial poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=camera_name),

        # Run a pose filter (6dof, 4dof or 1dof)
        Node(package='orca_filter', node_executable='pose_filter_node', output='screen',
             node_name='pose_filter_node', parameters=[pose_filter_node_params], remappings=[
                ('fcam_fp', '/' + camera_name + '/fp'),
            ])]

    return LaunchDescription(all_entities)
