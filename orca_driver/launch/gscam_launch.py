import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# GSCam launch, useful for testing the image processing pipeline
#
# Setup:
# RPi camera module 2 with a wide angle lens sold by BlueRobotics
# RPi Zero running the following command:
# raspivid --nopreview --mode 4 -w 800 -h 600 --framerate 15 --awb auto --brightness 55 --saturation 10 --sharpness 50 --contrast 15  -fl --timeout 0 --output - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.110 port=5600
#
# GSCam config:
cfg = 'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('orca_description'), 'urdf', 'orca.urdf')
    camera_info_url = 'file://' + get_package_share_directory('orca_driver') + '/cfg/wa1_dry_800x600.yaml'

    camera_name = 'left_camera'
    map_frame = 'map'
    base_link_frame = 'base_link'
    camera_frame = 'left_camera_frame'

    return LaunchDescription([
        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # GSCam node (provides video)
        Node(package='gscam', node_executable='gscam_node', output='screen',
             node_name='gscam_node', parameters=[{
                'gscam_config': cfg,
                'camera_name': 'wa1_dry',
                'camera_info_url': camera_info_url,
                'frame_id': camera_frame
            }], remappings=[
                ('camera/camera_info', '/' + camera_name + '/camera_info'),
                ('camera/image_raw', '/' + camera_name + '/image_raw'),
            ]),

        # # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen'),

        # AUV controller
        Node(package='orca_base', node_executable='base_node', output='screen',
             node_name='base_node', remappings=[
                ('filtered_odom', '/' + camera_name + '/base_odom')
            ]),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778  # Marker length
            }]),

        # Localizer
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=camera_name, parameters=[{
                'publish_tfs': 1,
                'publish_camera_pose': 0,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 1,
                'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
                'map_frame_id': map_frame,
                'camera_frame': camera_frame,
                'base_frame_id': base_link_frame,
                't_camera_base_x': 0.18,  # base_link_frame=>camera_frame
                't_camera_base_y': -0.15,
                't_camera_base_z': -0.0675,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': 3.14,
                't_camera_base_yaw': 1.57
            }]),
    ])
