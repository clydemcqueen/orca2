import os

import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Launch pool test #2


def generate_launch_description():
    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_description_path = get_package_share_directory('orca_description')
    urdf_path = os.path.join(orca_description_path, 'urdf', 'pt2.urdf')

    return LaunchDescription([
        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Forward camera
        Node(package='orca_driver', node_executable='opencv_camera_node', output='screen',
             node_name='opencv_camera_node', remappings=[
                ('image_raw', '/' + camera_name + '/image_raw'),
                ('camera_info', '/' + camera_name + '/camera_info'),
            ]),

        # Driver
        Node(package='orca_driver', node_executable='driver_node', output='screen',
             node_name='driver_node', parameters=[{
                'voltage_multiplier': 5.05,
                'thruster_4_reverse': True,  # Thruster 4 ESC is programmed incorrectly
                'tilt_channel': 6,
                'voltage_min': 0.0  # TODO
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', parameters=[{
                'auto_start': 0,  # Auto-start mission >= 5 TODO
                'auv_z_target': -0.5,
                'auv_xy_distance': 2.0
            }], remappings=[
                ('filtered_odom', '/' + camera_name + '/odom')
            ]),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,
                'marker_length': 0.1778,
                'make_not_use_map': 1,
                'map_init_style': 1,  # Init style 1: marker id and location is specified below:
                'map_init_id': 0,
                'map_init_pose_x': 0.0,
                'map_init_pose_y': 0.0,
                'map_init_pose_z': -0.5,
                'map_init_pose_roll': math.pi / 2,
                'map_init_pose_pitch': 0.0,
                'map_init_pose_yaw': -math.pi / 2
            }]),

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=camera_name, parameters=[{
                'publish_tfs': 1,
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'publish_image_marked': 0,
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': camera_frame,
                't_camera_base_x': 0.,
                't_camera_base_y': 0.063,
                't_camera_base_z': -0.16,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': -math.pi / 2,
                't_camera_base_yaw': math.pi / 2
            }]),
    ])
