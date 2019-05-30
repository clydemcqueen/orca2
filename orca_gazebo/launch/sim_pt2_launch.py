"""Simulate pool test #2"""

import os

import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'pt2.urdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'pt2.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'pt2_map.yaml')

    return LaunchDescription([
        # Launch Gazebo, loading orca.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the AUV to the simulation
        Node(package='orca_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', surface, '0']),

        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path], parameters=[{
                'use_sim_time': True,  # Use /clock if available
            }]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'auto_start': 6,  # Auto-start: 5 for keep station, 6 for random pattern
                'auv_z_target': -0.5,  # Waypoint z value
                'auv_xy_distance': 0.7  # Distance from waypoint to marker
            }], remappings=[
                ('filtered_odom', '/' + camera_name + '/odom')
            ]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0}]),  # Don't save a map to disk

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=camera_name, parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'base_odometry_pub_topic': 'odom',
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': camera_frame,
                't_camera_base_x': 0.,
                't_camera_base_y': 0.063,
                't_camera_base_z': -0.16,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': -math.pi / 2,
                't_camera_base_yaw': math.pi / 2
            }]),

        # Odometry filter takes camera pose, generates base_link odom, and publishes map to base_link tf
        # (Run w/o filter while PID tuning in pt2)
        # Node(package='odom_filter', node_executable='filter_node', output='screen',
        #      node_name='filter_node', node_namespace=camera_name, parameters=[{
        #         'use_sim_time': True,                       # Use /clock if available
        #         'sensor_frame': camera_frame,
        #     }]),
    ])
