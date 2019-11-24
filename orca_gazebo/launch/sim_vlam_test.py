"""
Launch a test harness for fiducial_vlam

Inject a new camera on-the-fly:
ros2 run orca_gazebo inject_entity.py install/orca_description/share/orca_description/urdf/fixed_camera.sdf 0 0 0 0 0 0

(But deleting a model from Gazebo might delete the ROS node before the ROS subscriptions, causing a memory leak.)
"""

import os

import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Must match camera name in URDF file
    forward_camera_name = 'forward_camera'
    forward_camera_frame = 'forward_camera_frame'
    down_camera_name = 'down_camera'
    down_camera_frame = 'down_camera_frame'

    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    forward_sdf_path = os.path.join(orca_gazebo_path, 'urdf', 'forward_camera.sdf')
    down_sdf_path = os.path.join(orca_gazebo_path, 'urdf', 'down_camera.sdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'vlam_test.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'vlam_test_map.yaml')

    return LaunchDescription([
        # Launch Gazebo, loading orca.world
        # Could use additional_env to add model path, but we need to add to the path, not replace it
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add forward-facing camera to the simulation
        Node(package='orca_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[forward_sdf_path, '0', '0', '0', '0', '0', '0']),

        # Add down-facing camera to the simulation
        Node(package='orca_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[down_sdf_path, '-0.2', '0', '0', '0', '1.570796', '0']),

        # # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0}]),  # Don't modify the map

        # Forward vloc
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=forward_camera_name, parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,
                'base_frame_id': '',  # Suppress publication of base_link tf
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': forward_camera_frame,
            }]),

        # Down vloc
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=down_camera_name, parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,
                'base_frame_id': '',  # Suppress publication of base_link tf
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': down_camera_frame,
            }]),
    ])
