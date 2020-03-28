import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Launch topside nodes


def generate_launch_description():
    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_description_path = get_package_share_directory('orca_description')
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    orca_driver_path = get_package_share_directory('orca_driver')
    params_path = os.path.join(orca_driver_path, 'launch', 'ft3_params.yaml')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_640x480.ini')  # TODO 1920x1080
    map_path = os.path.join(orca_driver_path, 'maps', 'simple_map.yaml')

    return LaunchDescription([
        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='log',
             arguments=[urdf_path]),

        # Forward camera
        Node(package='gscam2', node_executable='gscam_node', output='log',
             node_name='gscam_node', remappings=[
                ('image_raw', '/' + camera_name + '/image_raw'),
                ('camera_info', '/' + camera_name + '/camera_info'),
            ], parameters=[{
                'camera_info_path': camera_info_path,
            }]),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[{
            }], remappings=[
                ('control', 'rov_control'),  # Send control messages to auv_node
            ]),

        # Depth node, turns /barometer messages into /depth messages
        Node(package='orca_filter', node_executable='depth_node', output='screen',
             node_name='depth_node', parameters=[{
                'fluid_density': 997.0,
            }]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=camera_name, parameters=[
                params_path, {
                    'camera_frame_id': camera_frame,
                }]),

        # FP node, generate fiducial poses from observations and poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=camera_name, parameters=[{
            }]),

        # Filter
        Node(package='orca_filter', node_executable='filter_node', output='screen',
             node_name='filter_node', parameters=[params_path, {
                'urdf_file': urdf_path,
            }], remappings=[
                ('fcam_fp', '/' + camera_name + '/fp'),
            ]),

        # AUV controller
        Node(package='orca_base', node_executable='auv_node', output='screen',
             node_name='auv_node', parameters=[params_path, {
            }], remappings=[
                ('fcam_info', '/' + camera_name + '/camera_info'),
            ]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=camera_name),
    ])
