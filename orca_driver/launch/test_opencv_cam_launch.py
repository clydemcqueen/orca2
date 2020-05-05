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
    params_path = os.path.join(orca_driver_path, 'launch', 'ft3_params.yaml')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_640x480.ini')
    map_path = os.path.join(orca_driver_path, 'maps', 'ft3_map.yaml')

    return LaunchDescription([
        # Forward camera
        Node(package='opencv_cam', node_executable='opencv_cam_main', output='screen',
             node_name='opencv_cam_main', node_namespace=camera_name, parameters=[{
                'index': 200, # V4L index 0
                'codec': 'YUYV',
                'fps': 30,
                'width': 640,
                'height': 480,
                'camera_info_path': camera_info_path,
                'camera_frame_id': camera_frame,
            }]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length for new maps
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_node', node_namespace=camera_name, parameters=[
                params_path, {
                    'camera_frame_id': camera_frame,
                    'sub_camera_info_best_effort_not_reliable': 0,
                }]),

        # Measure lag
        # Node(package='pipe_perf', node_executable='image_sub_node', output='screen',
        #      node_name='image_sub_node', node_namespace=camera_name),
    ])
