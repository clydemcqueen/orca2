"""Launch a simulation with fiducial_vlam"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Sim time can be problematic, e.g., image messages are published with timestamps in ~100ms increments
    # Run everything in wall time until I get this figured out
    use_sim_time = False

    # Must match camera name in URDF file
    forward_camera_name = 'forward_camera'
    forward_camera_frame = 'forward_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    # There are two tested simulations:
    # -- ft3 (field test #3) is a 12' diameter pool with 12 markers arranged along the walls
    # -- large ring is a 20m diameter ring with 4 markers
    #    It relies on "move to marker" behavior which currently doesn't work with the filter_node
    ft3 = True

    if ft3:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3_map.yaml')
        params_path = os.path.join(orca_gazebo_path, 'launch', 'ft3_params.yaml')
    else:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring_map.yaml')
        params_path = os.path.join(orca_gazebo_path, 'launch', 'large_ring_params.yaml')

    all_entities = [
        # Launch Gazebo, loading the world
        # Could use additional_env to add model path, but we need to add to the path, not replace it
        ExecuteProcess(cmd=[
            # 'gazebo',
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the sub to the simulation
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', surface, '0', '0', '0']),

        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path], parameters=[{
                'use_sim_time': use_sim_time,
            }]),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': use_sim_time,
                'dev': '/dev/input/js0'  # Update as required
            }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[{
                'use_sim_time': use_sim_time,
            }], remappings=[
                ('control', 'rov_control'),  # Send control messages to auv_node
            ]),

        # Depth node, turns /barometer messages into /depth messages
        Node(package='orca_filter', node_executable='depth_node', output='screen',
             node_name='depth_node', parameters=[{
                'use_sim_time': use_sim_time,
                'fluid_density': 997.0,
            }]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
            }]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=forward_camera_name, parameters=[params_path, {
                'use_sim_time': use_sim_time,
                'camera_frame_id': forward_camera_frame,
            }]),

        # FP node, generate fiducial poses from observations and poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=forward_camera_name, parameters=[params_path, {
                'use_sim_time': use_sim_time,
            }]),

        # Filter
        Node(package='orca_filter', node_executable='filter_node', output='screen',
             node_name='filter_node', parameters=[params_path, {
                'use_sim_time': use_sim_time,
                'urdf_file': urdf_path,
            }], remappings=[
                ('fcam_fp', '/' + forward_camera_name + '/fp'),
            ]),

        # AUV controller
        Node(package='orca_base', node_executable='auv_node', output='screen',
             node_name='auv_node', parameters=[params_path, {
                'use_sim_time': use_sim_time,
            }], remappings=[
                ('fcam_info', '/' + forward_camera_name + '/camera_info'),
            ]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=forward_camera_name),
    ]

    return LaunchDescription(all_entities)
