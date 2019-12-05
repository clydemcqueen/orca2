import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# GSCam launch, useful for testing the image processing pipeline

# Setup:
# 2x RPi camera module 2 with a wide angle lens sold by BlueRobotics
# 2x RPi Zero running start_lcam.sh and start_rcam.sh

# GSCam config:
wa1_cfg = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'
wa2_cfg = 'udpsrc port=5602 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'


def generate_launch_description():
    output = 'screen'

    orca_description_path = get_package_share_directory('orca_description')
    orca_driver_path = get_package_share_directory('orca_driver')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    map_path = os.path.join(orca_driver_path, 'maps', 'simple_map.yaml')

    left_camera_info = 'file://' + get_package_share_directory('orca_driver') + '/cfg/wa1_dry_800x600.yaml'
    right_camera_info = 'file://' + get_package_share_directory('orca_driver') + '/cfg/wa2_dry_800x600.yaml'

    map_frame = 'map'

    # Must match the URDF file
    base_link_frame = 'base_link'
    left_camera_name = 'left_camera'
    left_camera_frame = 'left_camera_frame'
    right_camera_name = 'right_camera'
    right_camera_frame = 'right_camera_frame'

    return LaunchDescription([
        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output=output,
             arguments=[urdf_path]),

        # Left camera
        Node(package='gscam', node_executable='gscam_node', output=output,
             node_name='gscam_node', parameters=[{
                'gscam_config': wa1_cfg,
                'camera_name': left_camera_name,
                'camera_info_url': left_camera_info,
                'frame_id': left_camera_frame
            }], remappings=[
                ('camera/camera_info', '/' + left_camera_name + '/camera_info'),
                ('camera/image_raw', '/' + left_camera_name + '/image_raw'),
            ]),

        # Right camera
        Node(package='gscam', node_executable='gscam_node', output=output,
             node_name='gscam_node', parameters=[{
                'gscam_config': wa2_cfg,
                'camera_name': right_camera_name,
                'camera_info_url': right_camera_info,
                'frame_id': right_camera_frame
            }], remappings=[
                ('camera/camera_info', '/' + right_camera_name + '/camera_info'),
                ('camera/image_raw', '/' + right_camera_name + '/image_raw'),
            ]),

        # Mapper
        Node(package='fiducial_vlam', node_executable='vmap_node', output=output,
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map -- left camera
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_left', node_namespace=left_camera_name, parameters=[{
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': 0,
                'camera_frame_id': left_camera_frame,
            }]),

        # Localize against the map -- right camera
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_right', node_namespace=right_camera_name, parameters=[{
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,  # Turn off per-marker TFs, too noisy
                'sub_camera_info_best_effort_not_reliable': 1,
                'publish_camera_pose': 1,
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,
                'stamp_msgs_with_current_time': 0,
                'camera_frame_id': right_camera_frame,
            }]),

        # Filter
        Node(package='orca_filter', node_executable='filter_node', output=output,
             node_name='filter_node', parameters=[{
                'param_fluid_density': 997.0,
                'baro_init': 0,  # Init in-air
                'predict_accel': False,
                'predict_accel_control': False,
                'predict_accel_drag': False,
                'predict_accel_buoyancy': False,
                'filter_baro': False,
                'filter_fcam': False,
                'filter_lcam': True,
                'filter_rcam': True,
                'urdf_file': urdf_path,
                'urdf_barometer_joint': 'baro_joint',
                'urdf_left_camera_joint': 'left_camera_frame_joint',
                'urdf_right_camera_joint': 'right_camera_frame_joint',
            }], remappings=[
                ('lcam_f_map', '/' + left_camera_name + '/camera_pose'),
                ('rcam_f_map', '/' + right_camera_name + '/camera_pose'),
            ]),
    ])
