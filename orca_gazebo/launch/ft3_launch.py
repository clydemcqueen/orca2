"""Simulate field test #3"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Sim time can be problematic, e.g., image messages are published with timestamps in ~100ms increments
    # Run everything in wall time until I get this figured out
    use_sim_time = False

    # Update: I hacked up several sensor plugins in gazebo_ros_pkgs to publish everything in wall time,
    # so vloc_node doesn't need to overwrite the timestamps
    stamp_msgs_with_current_time = 0

    # Must match camera name in URDF file
    forward_camera_name = 'forward_camera'
    forward_camera_frame = 'forward_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3_map.yaml')
    auv_node_params_path = os.path.join(orca_gazebo_path, 'launch', 'auv_node_params.yaml')
    filter_node_params_path = os.path.join(orca_gazebo_path, 'launch', 'filter_node_params.yaml')

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
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', surface, '0', '0', '0']),

        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path], parameters=[{
                'use_sim_time': use_sim_time,
            }]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': use_sim_time,
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # ROV controller
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[{
                'use_sim_time': use_sim_time,
            }], remappings=[
            ]),

        # Depth node, turns /barometer messages into /depth messages
        Node(package='orca_filter', node_executable='depth_node', output='screen',
             node_name='depth_node', parameters=[{
                'use_sim_time': use_sim_time,
                'fluid_density': 997.0,
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='auv_node', output='screen',
             node_name='auv_node', parameters=[auv_node_params_path, {
                'use_sim_time': use_sim_time,
            }], remappings=[
                ('fcam_f_map', '/' + forward_camera_name + '/camera_pose'),
                ('fcam_image', '/' + forward_camera_name + '/image_raw'),
                ('fcam_info', '/' + forward_camera_name + '/camera_info'),
            ]),

        # Filter
        # Node(package='orca_filter', node_executable='filter_node', output='screen',
        #      node_name='filter_node', parameters=[filter_node_params_path, {
        #         'use_sim_time': use_sim_time,
        #         'urdf_file': urdf_path,
        #     }], remappings=[
        #         ('fcam_f_map', '/' + forward_camera_name + '/camera_pose'),
        #     ]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map -- forward camera
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=forward_camera_name, parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 0,  # Don't publish t_map_base, do this in filter_node or auv_node
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': stamp_msgs_with_current_time,
                'camera_frame_id': forward_camera_frame,
            }]),
    ])
