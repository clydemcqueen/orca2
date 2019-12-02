"""Launch a simulation"""

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
    left_camera_name = 'left_camera'
    left_camera_frame = 'left_camera_frame'
    right_camera_name = 'right_camera'
    right_camera_frame = 'right_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    world_path = os.path.join(orca_gazebo_path, 'worlds', 'huge.world')
    map_path = os.path.join(orca_gazebo_path, 'worlds', 'huge_map.yaml')

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
             arguments=[urdf_path, '0', '0', surface, '0', '0', '0']),

        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path], parameters=[{
                'use_sim_time': True,  # Use /clock if available
            }]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': use_sim_time,
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', parameters=[{
                'use_sim_time': use_sim_time,
                'param_fluid_density': 997.0,
                'auto_start': 0,  # Auto-start AUV mission
                'auv_controller': 5,  # DepthController
                'auv_z_target': -2.5,
            }], remappings=[
                # ('odom', '/' + left_camera_name + '/odom'),
                # ('odom', '/filtered_odom'),
            ]),

        # Filter
        Node(package='orca_filter', node_executable='filter_node', output='screen',
             node_name='filter_node', parameters=[{
                'use_sim_time': use_sim_time,
                'param_fluid_density': 997.0,
                'baro_init': 0,  # Init in-air
                'predict_accel': False,
                'predict_accel_control': False,
                'predict_accel_drag': False,
                'predict_accel_buoyancy': False,
                'filter_baro': True,
                'filter_fcam': False,
                'filter_lcam': True,
                'filter_rcam': True,
                'urdf_file': urdf_path,
                'urdf_barometer_joint': 'baro_joint',
                'urdf_forward_camera_joint': 'forward_camera_frame_joint',
                'urdf_left_camera_joint': 'left_camera_frame_joint',
                'urdf_right_camera_joint': 'right_camera_frame_joint',
            }], remappings=[
                ('fcam_f_map', '/' + forward_camera_name + '/camera_pose'),
                ('lcam_f_map', '/' + left_camera_name + '/camera_pose'),
                ('rcam_f_map', '/' + right_camera_name + '/camera_pose'),
            ]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map -- forward camera
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_forward', node_namespace=forward_camera_name, parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': int(not use_sim_time),
                'camera_frame_id': forward_camera_frame,
            }]),

        # Localize against the map -- left camera
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_left', node_namespace=left_camera_name, parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': stamp_msgs_with_current_time,
                'camera_frame_id': left_camera_frame,
            }]),

        # Localize against the map -- right camera
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_right', node_namespace=right_camera_name, parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': stamp_msgs_with_current_time,
                'camera_frame_id': right_camera_frame,
            }]),
    ])
