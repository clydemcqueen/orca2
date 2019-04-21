import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# GSCam launch, useful for testing the image processing pipeline
#
# Setup:
# RPi camera module 2 with a wide angle lens sold by BlueRobotics
# RPi Zero running the following command:
#     raspivid --nopreview --mode 5 --bitrate 15000000 --intra 1 --awb auto --brightness 55 --saturation 10 \
#       --sharpness 50 --contrast 15  -fl --timeout 0 --output - | \
#       gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=<host> port=5600
#
# GSCam config:
cfg = 'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'


def generate_launch_description():
    # TODO move orca.urdf from orca_gazebo to orca_description, and include it here
    orca_driver_path = get_package_share_directory('orca_driver')
    urdf_path = os.path.join(orca_driver_path, 'cfg', 'orca.urdf')
    camera_info_url = 'file://' + orca_driver_path + '/cfg/wa1_dry.yaml'
    camera_frame_id = 'left_camera_frame' # Must match frame id in the URDF file

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
                'frame_id': camera_frame_id
            }], remappings=[
                ('camera/camera_info', '/camera_info'),
                ('camera/image_raw', '/image_raw'),
            ]),

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
                'camera_frame_id': camera_frame_id,         # Set frame id (not the same as the link name)
                't_camera_base_x': 0.18,                    # base_link=>camera_frame_id
                't_camera_base_y': -0.15,
                't_camera_base_z': -0.0675,
                't_camera_base_roll': 0.,
                't_camera_base_pitch': 3.14,
                't_camera_base_yaw': 1.57

            }]),
    ])
