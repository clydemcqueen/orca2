"""Launch a simulation, with all the bells and whistles"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Must match camera name in URDF file
    left_camera_name = 'left_camera'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'small.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'small_map.yaml')

    return LaunchDescription([
        # Launch Gazebo, loading orca.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',      # Publish /clock
            '-s', 'libgazebo_ros_factory.so',   # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the AUV to the simulation
        Node(package='orca_gazebo', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', surface, '0']),

        # Publish base_link=>left_camera_link
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path], parameters=[{
                'use_sim_time': True,                       # Use /clock if available
            }]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': True,                       # Use /clock if available
                'dev': '/dev/input/js0'                     # Update as required
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', parameters=[{
                'use_sim_time': True,                       # Use /clock if available
            }], remappings=[
                ('filtered_odom', '/' + left_camera_name + '/base_odom')
            ]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': True,                       # Use /clock if available
                'publish_tfs': 1,                           # Publish marker /tf
                'marker_length': 0.1778,                    # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0}]),                   # Don't save a map to disk

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=left_camera_name, parameters=[{
                'use_sim_time': True,                       # Use /clock if available
                'publish_tfs': 1,                           # Publish map=>base_link and map=>left_camera_frame
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'stamp_msgs_with_current_time': 0,          # Use incoming message time, not now()
                'camera_frame_id': 'left_camera_frame',     # left_camera_frame != left_camera_link
                't_camera_base_x': 0.18,                    # base_link=>left_camera_frame
                't_camera_base_y': -0.15,
                't_camera_base_z': -0.0675,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': 3.14,
                't_camera_base_yaw': 1.57
            }]),

        # Odometry filter takes camera pose, generates base_link odom, and publishes map to base_link tf
        # Node(package='orca_base', node_executable='filter_node', output='screen',
        #      node_name='filter_node', node_namespace=left_camera_name, parameters=[{
        #         'use_sim_time': True,                       # Use /clock if available
        #         't_camera_base_x': -0.2,
        #         't_camera_base_y': 0.,
        #         't_camera_base_z': 0.,
        #         't_camera_base_roll': 1.57,
        #         't_camera_base_pitch': 3.14,
        #         't_camera_base_yaw': 0.}]),
    ])
