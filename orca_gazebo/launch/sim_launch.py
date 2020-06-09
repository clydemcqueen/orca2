"""Launch a simulation with fiducial_vlam"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Sim time can be problematic, e.g., image messages are published with timestamps in ~100ms increments
    # I hacked up several sensor plugins in gazebo_ros_pkgs to publish everything in wall time

    # Must match camera name in URDF file
    forward_camera_name = 'forward_camera'
    forward_camera_frame = 'forward_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    # URDF file specifies camera resolution, see orca_description
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    # How far away from a single marker is a good pose?
    # 800x600: 2.0
    # 1920x1280: 4.0
    good_pose_dist = 2.0

    # How far away from a single marker is a good observation (just bearing & distance)?
    # 800x600: 10.0
    # 1920x1280: 14.0
    good_obs_dist = 10.0

    # There are two tested simulations:
    # -- ft3 (field test #3) is a 12' diameter pool with 12 markers arranged along the walls
    # -- large ring is a 20m diameter ring with 4 markers
    ft3 = True

    if ft3:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'ft3_map.yaml')

        # Do not allow MTM recovery
        global_plan_allow_mtm = False

        # If xy distance is > this, then build a long plan (rotate, run, rotate)
        # Otherwise build a short plan (move in all DoF at once)
        pose_plan_max_short_plan_xy = 0.5

        # The pool is a bit cramped, so get closer to the markers
        pose_plan_target_dist = 0.8
    else:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring_map.yaml')

        # MTM recovery is essential over these long distances
        global_plan_allow_mtm = True

        # Allow for longer short (all DoF) plans -- keeps the camera on the marker
        pose_plan_max_short_plan_xy = 2.0

        # Move to within 1m of the marker -- a good default
        pose_plan_target_dist = 1.0

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
                # Publish marker /tf
                'publish_tfs': 1,

                # Marker length for new maps
                'marker_length': 0.1778,

                # Load a pre-built map from disk
                'marker_map_load_full_filename': map_path,

                # Do not modify the map
                'make_not_use_map': 0
            }]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=forward_camera_name, parameters=[{
                'camera_frame_id': forward_camera_frame,
                'publish_tfs': 0,
                'publish_tfs_per_marker': 0,

                # Gazebo publishes camera info best-effort
                'sub_camera_info_best_effort_not_reliable': 1,

                # Publish the camera pose
                'publish_camera_pose': 1,

                # Don't publish anything else
                'publish_base_pose': 0,
                'publish_camera_odom': 0,
                'publish_base_odom': 0,

                # Keep the existing timestamps
                'stamp_msgs_with_current_time': 0,
            }]),

        # FP node, generate fiducial poses from observations and poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=forward_camera_name, parameters=[{
            }]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=forward_camera_name),
    ]

    auv_node_params = {
        'param_fluid_density': 997.0,

        # Timer (mode 0) is stable w/ or w/o filter
        'loop_driver': 0,

        # Either auv_node or filter_node should publish tf, but not both
        'publish_tf': True,

        # Do not allow waypoints
        'pose_plan_waypoints': False,

        'good_pose_dist': good_pose_dist,
        'good_obs_dist': good_obs_dist,
        'global_plan_allow_mtm': global_plan_allow_mtm,
        'pose_plan_max_short_plan_xy': pose_plan_max_short_plan_xy,
        'pose_plan_target_dist': pose_plan_target_dist,
        'mtm_plan_target_dist': 1.5,
    }

    # Run orca_filter or not
    filter_poses = True

    if filter_poses:
        all_entities.append(
            Node(package='orca_filter', node_executable='filter_node', output='screen',
                 node_name='filter_node', parameters=[{
                    'urdf_file': urdf_path,
                    'param_fluid_density': 997.0,
                    'predict_accel': False,
                    'predict_accel_control': False,
                    'predict_accel_drag': False,
                    'predict_accel_buoyancy': False,
                    'filter_baro': True,
                    'filter_fcam': True,
                    'publish_tf': False,
                    'good_pose_dist': 2.0,
                    'good_obs_dist': 10.0,
                }], remappings=[
                    ('fcam_fp', '/' + forward_camera_name + '/fp'),
                ]))
        all_entities.append(
            Node(package='orca_base', node_executable='auv_node', output='screen',
                 node_name='auv_node', parameters=[auv_node_params], remappings=[
                    ('filtered_fp', 'filtered_fp'),
                ]))
    else:
        all_entities.append(
            Node(package='orca_base', node_executable='auv_node', output='screen',
                 node_name='auv_node', parameters=[auv_node_params], remappings=[
                    ('filtered_fp', 'fp'),
                ]))

    return LaunchDescription(all_entities)
