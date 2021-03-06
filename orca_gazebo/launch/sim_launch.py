#!/usr/bin/env python3

# Copyright (c) 2020, Clyde McQueen.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch a simulation with fiducial_vlam."""

from enum import Enum
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# There are several tested worlds:
class World(Enum):
    SMALL_FIELD = 0  # 12' diameter pool with a field of 6 markers arranged in a 2x3 vertical field
    SMALL_RING = 1  # 12' diameter pool with 12 markers arranged along the walls
    LARGE_RING = 2  # large_ring is a 20m diameter ring with 4 markers
    TWO_WALL = 3  # Two markers on the wall
    SIX_RING = 4  # 12' diameter pool with 6 markers arranged along the walls


def generate_launch_description():
    # Sim time can be problematic, e.g., image messages are published with timestamps in ~100ms
    # increments. I hacked up several sensor plugins in gazebo_ros_pkgs to publish everything in
    # wall time.

    # Must match camera name in URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    # The AUV must be injected at the surface to calibrate the barometer
    surface = '0'

    orca_description_path = get_package_share_directory('orca_description')
    orca_gazebo_path = get_package_share_directory('orca_gazebo')

    # URDF file specifies camera resolution, see orca_description
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    # How far away from a single marker is a good pose?
    # 800x600: 2.0
    # 1920x1280: 4.0
    good_pose_dist = 2.5

    # How far away from a single marker is a good observation (just bearing & distance)?
    # 800x600: 10.0
    # 1920x1280: 14.0
    good_obs_dist = 10.0

    # Run pose_filter_node or not
    filter_poses = False

    # Select map
    world = World.SMALL_RING

    if world == World.SMALL_FIELD:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'small_field.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'small_field_map.yaml')

        # Do not allow MTM recovery
        global_plan_allow_mtm = True  # TODO ft3 hack

        # If xy distance is > this, then build a long plan (rotate, run, rotate)
        # Otherwise build a short plan (move in all DoF at once)
        pose_plan_max_short_plan_xy = 0.5

        # The pool is a bit cramped, so get closer to the markers
        pose_plan_target_dist = 0.8
    elif world == World.SMALL_RING:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'small_ring.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'small_ring_map.yaml')
        global_plan_allow_mtm = False
        pose_plan_max_short_plan_xy = 0.5
        pose_plan_target_dist = 0.8
    elif world == World.SIX_RING:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'six_ring.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'six_ring_map.yaml')
        global_plan_allow_mtm = False
        pose_plan_max_short_plan_xy = 0.5
        pose_plan_target_dist = 0.8
    elif world == World.LARGE_RING:
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'large_ring_map.yaml')

        # MTM recovery is essential over these long distances
        global_plan_allow_mtm = True

        # Allow for longer short (all DoF) plans -- keeps the camera on the marker
        pose_plan_max_short_plan_xy = 2.0

        # Move to within 1m of the marker -- a good default
        pose_plan_target_dist = 1.0
    else:  # world == World.TWO_WALL
        world_path = os.path.join(orca_gazebo_path, 'worlds', 'two_wall.world')
        map_path = os.path.join(orca_gazebo_path, 'worlds', 'two_wall_map.yaml')
        global_plan_allow_mtm = False
        pose_plan_max_short_plan_xy = 0.5
        pose_plan_target_dist = 0.8

    # Optionally build and use a map
    build_map = True
    use_built_map = False

    if build_map:
        vmap_node_params = {
            # Publish marker /tf
            'psm_publish_tfs': 1,

            # Marker length
            'map_marker_length': 0.1778,

            # Don't load a map
            'map_load_filename': '',

            # Save the map
            'map_save_filename': 'sam_map',
        }
        if world == World.SMALL_FIELD:
            vmap_node_params.update({
                # Init map from a marker pose
                # Leave roll, pitch, yaw at defaults
                'map_init_style': 1,
                'map_init_id': 0,
                'map_init_pose_x': 2.0,
                'map_init_pose_y': 0.6,
                'map_init_pose_z': 0.3,
            })
        elif world == World.SMALL_RING:
            vmap_node_params.update({
                'map_init_style': 1,
                'map_init_id': 0,
                'map_init_pose_x': 1.8,
                'map_init_pose_y': 0.0,
                'map_init_pose_z': -0.5,
                # 'map_init_style': 2,
                # 'map_init_id': 0,
                # 'map_init_pose_x': 0.16,
                # 'map_init_pose_y': 0.0,
                # 'map_init_pose_z': -0.062,  # -0.125 (baro.hpp) + 0.063 (orca.urdf.xacro),
                # 'map_init_pose_roll': -math.pi/2,
                # 'map_init_pose_pitch': 0.0,
                # 'map_init_pose_yaw': -math.pi/2,
            })
        elif world == World.SIX_RING:
            vmap_node_params.update({
                'map_init_style': 1,
                'map_init_id': 0,
                'map_init_pose_x': 1.8,
                'map_init_pose_y': 0.0,
                'map_init_pose_z': -0.5,
            })
        elif world == World.LARGE_RING:
            vmap_node_params.update({
                'map_init_style': 1,
                'map_init_id': 0,
                'map_init_pose_x': 8.0,
                'map_init_pose_y': 0.0,
                'map_init_pose_z': -0.5,
            })
        else:  # world == World.TWO_WALL
            vmap_node_params.update({
                'map_init_style': 1,
                'map_init_id': 0,
                'map_init_pose_x': 1.5,
                'map_init_pose_y': 0.0,
                'map_init_pose_z': -0.5,
            })
    else:
        vmap_node_params = {
            # Publish marker /tf
            'psm_publish_tfs': 1,

            # Load map
            'map_load_filename': 'sam_map' if use_built_map else map_path,

            # Don't save the map
            'map_save_filename': '',
        }

    fp_node_params = {
        # Publish map=>base tf if we're not running a filter
        'publish_tf': not filter_poses,
    }

    model_params = {
        # Match orca.urdf (slight positive buoyancy):
        'mdl_mass': 9.9,
        'mdl_volume': 0.01,
        'mdl_fluid_density': 997.0,
        'mdl_thrust_scale': 1.0,  # Simulated thrust curve is perfectly linear
    }

    rov_node_params = {
        # Moved to defaults in orca_base::ROVContext
    }
    rov_node_params.update(model_params)

    pose_filter_node_params = {
        # Hack: predicting acceleration makes the filter kinda stable
        # TODO(clyde): huge amount of work needed here
        'predict_accel': True,
        'predict_accel_control': True,
        'predict_accel_drag': True,
        'predict_accel_buoyancy': True,
        'filter_baro': False,  # Fuse depth
        'filter_fcam': True,
        'publish_tf': True,  # Publish map=>base tf
        'good_pose_dist': good_pose_dist,
        'good_obs_dist': good_obs_dist,

        # Turn outlier detection off
        'ukf_outlier_distance': -1.0,
    }
    pose_filter_node_params.update(model_params)

    auv_node_params = {
        # Timer (mode 0) is stable w/ or w/o filter
        'loop_driver': 0,

        # If we're not running a filter, then override depth in auv_node
        'depth_override': not filter_poses,

        # Do not allow waypoints
        'pose_plan_waypoints': False,

        # Leave room for the filter to flop between depth and fp
        'timeout_fp_ms': 500,

        'good_pose_dist': good_pose_dist,
        'good_obs_dist': good_obs_dist,
        'global_plan_allow_mtm': global_plan_allow_mtm,
        'pose_plan_max_short_plan_xy': pose_plan_max_short_plan_xy,
        'pose_plan_target_dist': pose_plan_target_dist,
        'mtm_plan_target_dist': 1.5,

        # Hacks for PID tuning
        # 'control_use_est_yaw': True,
        # 'global_plan_max_xy_err': 3.0,

        # TODO these screw up the simulation...
        # rov_pressure_pid_k? * 10000 (roughly 1/Pascals to 1/meters)
        # 'auv_z_pid_kp': 0.6,
        # 'auv_z_pid_ki': 0.2,
        # 'auv_z_pid_kd': 0.45,

        # classic(10., 2.5)
        # 'auv_yaw_pid_kp': 6.0,
        # 'auv_yaw_pid_ki': 4.8,
        # 'auv_yaw_pid_kd': 1.875,

        # no_overshoot(4.0, 4.0)
        # 'auv_x_pid_kp': 0.8,
        # 'auv_x_pid_ki': 0.4,
        # 'auv_x_pid_kd': 1.0672,

        # no_overshoot(4.0, 4.0)
        # 'auv_y_pid_kp': 0.8,
        # 'auv_y_pid_ki': 0.4,
        # 'auv_y_pid_kd': 1.0672,
    }
    auv_node_params.update(model_params)

    all_entities = [
        # Launch Gazebo, loading the world
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
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen',
             node_name='robot_state_publisher', arguments=[urdf_path]
             ),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
             }]),

        # Barometer filter
        Node(package='orca_filter', node_executable='baro_filter_node', output='screen',
             parameters=[{
                 'ukf_Q': True,
             }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[rov_node_params], remappings=[
                ('control', 'rov_control'),  # Send control messages to auv_node
                ('barometer', 'filtered_barometer'),  # Use filtered barometer messages
             ]),

        # Depth node, turns /barometer messages into /depth messages
        Node(package='orca_filter', node_executable='depth_node', output='screen',
             node_name='depth_node', parameters=[model_params], remappings=[
                ('fp', '/' + camera_name + '/fp'),
                ('barometer', 'filtered_barometer'),  # Use filtered barometer messages
             ]),

        # Publish, and possibly build, a map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[vmap_node_params]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_forward', node_namespace=camera_name, parameters=[{
                # Localize, don't calibrate
                'loc_calibrate_not_loocalize': 0,

                # 0: OpenCV, 1: GTSAM
                'loc_camera_sam_not_cv': 1,

                'psl_camera_frame_id': camera_frame,
                'psl_publish_tfs': 0,

                # 0: DICT_4x4_50 (default)
                # 8: DICT_6X6_250
                'loc_aruco_dictionary_id': 8,

                # Gazebo publishes camera info best-effort
                'psl_sub_camera_info_best_effort_not_reliable': 1,

                # Publish the camera pose
                'psl_publish_camera_pose': 1,

                # Don't publish anything else
                'psl_publish_base_pose': 0,
                'psl_publish_camera_odom': 0,
                'psl_publish_base_odom': 0,

                # Keep the existing timestamps
                'psl_stamp_msgs_with_current_time': 0,
             }]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=camera_name),

        # Combine poses and observations into fiducial poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=camera_name, parameters=[fp_node_params]),
    ]

    if filter_poses:
        all_entities.append(
            Node(package='orca_filter', node_executable='pose_filter_node', output='screen',
                 node_name='pose_filter_node', parameters=[pose_filter_node_params], remappings=[
                    ('fcam_fp', '/' + camera_name + '/fp'),
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
                    ('filtered_fp', '/' + camera_name + '/fp'),
                 ]))

    return LaunchDescription(all_entities)
