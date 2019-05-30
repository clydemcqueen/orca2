import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Orca launch


def generate_launch_description():
    # Must match camera name in URDF file
    main_camera_name = 'left_camera'  # TODO

    urdf = os.path.join(get_package_share_directory('orca_description'), 'urdf', 'orca.urdf')

    return LaunchDescription([
        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf]),

        # Main camera
        Node(package='orca_driver', node_executable='opencv_camera_node', output='screen',
             node_name='opencv_camera_node', remappings=[
                ('image_raw', '/' + main_camera_name + '/image_raw'),
                ('camera_info', '/' + main_camera_name + '/camera_info'),
            ]),

        # Bar30 node
        Node(package='orca_driver', node_executable='bar30_node.py', output='screen'),

        # Driver node handles thrusters, lights and camera tilt
        Node(package='orca_driver', node_executable='driver_node', output='screen',
             node_name='driver_node', parameters=[{
                'voltage_multiplier': 5.05,
                'thruster_4_reverse': True,  # Thruster 4 ESC is programmed incorrectly
                'tilt_channel': 6,
                'voltage_min': 0.0  # 0.0 For bench testing TODO
            }]),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', parameters=[{
                'auto_mission': False  # False for bench testing TODO
            }], remappings=[
                ('filtered_odom', '/' + main_camera_name + '/base_odom')
            ]),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778  # Marker length
            }]),

        # TODO add odom_filter

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=main_camera_name, parameters=[{
                'publish_tfs': 1,  # Publish map=>base_link and map=>left_camera_frame
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'camera_frame_id': 'left_camera_frame',  # left_camera_frame != left_camera_link
                't_camera_base_x': 0.18,  # base_link=>left_camera_frame
                't_camera_base_y': -0.15,
                't_camera_base_z': -0.0675,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': 3.14,
                't_camera_base_yaw': 1.57
            }]),
    ])
