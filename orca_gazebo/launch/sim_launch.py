"""Launch a generic simulation, with all the bells and whistles"""

import os

import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Must match camera name in URDF file
    left_camera_name = 'left_camera'
    left_camera_frame = 'left_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'large.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'large_map.yaml')

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
                'auto_start': 3,  # Auto-start AUV mission
                'auv_z_target': -2.0,  # Mission runs 2m below the surface
                'auv_controller': 3,  # Slow controller
                'auv_epsilon_xy': 0.05,
                'auv_epsilon_z': 0.05,
                'auv_epsilon_yaw': 0.1,
                'auv_jerk_xy': 10.0,
                'auv_jerk_z': 10.0,
                'auv_jerk_yaw': 20.0,

            }], remappings=[
                ('filtered_odom', '/' + left_camera_name + '/odom')
            ]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0}]),  # Don't modify the map

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=left_camera_name, parameters=[{
                'use_sim_time': True,  # Use /clock if available
                'publish_tfs': 1,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'base_odometry_pub_topic': 'odom',
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': left_camera_frame,
                't_camera_base_x': 0.18,
                't_camera_base_y': -0.15,
                't_camera_base_z': -0.0675,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': math.pi,
                't_camera_base_yaw': math.pi / 2
            }]),
    ])
