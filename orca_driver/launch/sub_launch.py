import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Launch sub nodes


def generate_launch_description():
    # Camera name must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'
    fps = 30
    size = '1920x1080'

    orca_driver_path = get_package_share_directory('orca_driver')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_dry_' + size + '.ini')

    return LaunchDescription([
        Node(package='orca_driver', node_executable='barometer_node', output='screen', node_name='barometer_node'),

        Node(package='orca_driver', node_executable='driver_node', output='screen', node_name='driver_node',
             parameters=[{
                 'thruster_0_channel': 0,
                 'thruster_1_channel': 1,
                 'thruster_2_channel': 4,
                 'thruster_3_channel': 5,
                 'thruster_4_channel': 6,
                 'thruster_5_channel': 7,
             }]),

        Node(package='h264_image_transport', node_executable='h264_cam_node', output='screen',
             node_name='h264_cam_node', node_namespace=camera_name, parameters=[{
                'input_fn': '/dev/video1',
                'fps': fps,
                'size': size,
                'frame_id': camera_frame,
                'camera_info_path': camera_info_path,
            }]),
    ])
