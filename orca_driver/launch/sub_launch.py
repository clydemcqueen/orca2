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

    orca_driver_path = get_package_share_directory('orca_driver')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_640x480.ini')


    return LaunchDescription([
        Node(package='orca_driver', node_executable='barometer_node', output='screen', node_name='barometer_node'),

        Node(package='orca_driver', node_executable='driver_node', output='screen', node_name='driver_node',
             parameters=[{
                 'thruster_4_reverse': True  # Thruster 4 ESC is programmed incorrectly TODO ?
             }]),

        Node(package='h264_image_transport', node_executable='v4l_cam_node', output='screen',
             node_name='v4l_cam_node', node_namespace=camera_name, parameters=[{
                'input_fn': '/dev/video2',
                'fps': '30',
                'size': '640x480',
                'frame_id': camera_frame,
                'camera_info_path': camera_info_path,
            }]),
    ])
