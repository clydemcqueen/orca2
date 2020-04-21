import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Useful for testing a single camera pipeline


def generate_launch_description():
    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_driver_path = get_package_share_directory('orca_driver')
    camera_info_url = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_640x480.ini')  # TODO 1920x1080

    # v4l camera => ROS images
    cfg_v4l_ros = 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert'

    # v4l camera => mp4 fil and ROS images
    cfg_v4l_rec_and_ros = 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! tee name=fork ! queue ! mp4mux ! filesink location=save.mp4 fork. ! avdec_h264 ! videoconvert'

    # udp stream => ROS images
    cfg_rcv_ros = 'udpsrc port=5600 ! queue ! application/x-rtp ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'

    # v4l camera => udp stream and ROS images
    cfg_v4l_snd_and_ros = 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! tee name=fork ! queue ! rtph264pay config-interval=10 ! udpsink host=127.0.0.1 port=5600 fork. ! queue ! avdec_h264 ! videoconvert'

    # v4l camera => compressed ROS images
    cfg_v4l_jpeg_ros = 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! jpegenc'

    return LaunchDescription([
        # Forward camera
        Node(package='gscam', node_executable='gscam_main', output='screen',
             node_name='gscam_node2', node_namespace=camera_name, remappings=[
                ('image_raw', '/' + camera_name + '/image_raw'),
                ('camera_info', '/' + camera_name + '/camera_info'),
            ], parameters=[{
                'gscam_config': cfg_v4l_jpeg_ros,
                'image_encoding': 'jpeg',
                'preroll': True, # Forces pipeline to negotiate early, catching errors
                'camera_info_url': camera_info_url,
                'camera_name': camera_name,
                'frame_id': camera_frame,
            }]),

        # Forward camera
        # Node(package='gscam', node_executable='gscam_main', output='screen',
        #      node_name='gscam_node2', node_namespace=camera_name, remappings=[
        #         ('image_raw', '/' + camera_name + '/image_raw2'),
        #         ('camera_info', '/' + camera_name + '/camera_info2'),
        #     ], parameters=[{
        #         'gscam_config': cfg_rcv_ros,
        #         'camera_info_url': camera_info_url,
        #         'camera_name': camera_name,
        #         'frame_id': camera_frame,
        #     }]),
    ])
