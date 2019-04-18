import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# USB camera launch, useful for testing the image processing pipeline


def generate_launch_description():
    # TODO move orca.urdf from orca_gazebo to orca_description, and include it here
    urdf = os.path.join(get_package_share_directory('orca_driver'), 'cfg', 'orca.urdf')

    return LaunchDescription([
        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf]),

        # USB camera node (provides video)
        Node(package='orca_driver', node_executable='usb_camera_node', output='screen'),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,                           # Publish marker /tf
                'marker_length': 0.1778                     # Marker length
            }]),

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', parameters=[{
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
    ])
