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

"""
Run experiments to calculate drag coefficients.

Usage:
-- set drag coefficients to some default values
-- experiments will turn off PIDs and run various motions
-- compare planned vs actual trajectory to calculate new drag coefficients
-- simulated with "small_field" world

With filter:
ros2 run orca_base drag_experiment.py

Without filter:
ros2 run orca_base drag_experiment.py --ros-args -r filtered_fp:=/forward_camera/fp
"""

from typing import Optional

from geometry_msgs.msg import Point, Pose
from mission_experiment import MissionExperiment, MissionExperimentRunNode
from orca_msgs.msg import Control, MissionState
from orca_util import get_quaternion, get_yaw, norm_angle, seconds
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import rclpy
import rclpy.logging

# Run velocities
auv_xy_velo = 0.4
auv_z_velo = 0.2  # Run slower to actually hit the run phase
auv_yaw_velo = 0.4

# Use these parameters to force a simple back/forth motion
straight_motion_auv_params = [
    # pose_plan_max_short_plan_xy controls how much distance to cover with an "all DoF" motion,
    # vs. the more typical "long plan" that turns toward the motion, runs, then turns again.
    # Set this to a large value so that we can run backwards and side-to-side without turning.
    Parameter(name='pose_plan_max_short_plan_xy',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=99.)),

    # Turn off replanning
    Parameter(name='global_plan_max_xy_err',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=99.0)),

    # good_pose_dist controls how close we need to be to a marker to trust the pose.
    # Set this to a large-ish value so that we always build a local plan at each pose.
    Parameter(name='good_pose_dist',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=4.)),

    # Specify the run velocities
    Parameter(name='auv_xy_velo',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=auv_xy_velo)),
    Parameter(name='auv_z_velo',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=auv_z_velo)),
    Parameter(name='auv_yaw_velo',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                   double_value=auv_yaw_velo)),
]

# Turn off PID controllers
no_pids_auv_params = [
    Parameter(name='auv_pid_enabled',
              value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)),
]



# Turn on PID controllers
yes_pids_auv_params = [
    Parameter(name='auv_pid_enabled',
              value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
]

# Drag coefficients
drag_coef_f = 0.80  # After 1st round drag tests
drag_coef_s = 0.95
drag_coef_z = 0.95
drag_partial_const_yaw = 0.004
fluid_density = 997.
drag_coef_yaw = fluid_density * drag_partial_const_yaw

# Set drag coefficients
drag_auv_params = [
    Parameter(name='mdl_drag_coef_f',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=drag_coef_f)),
    Parameter(name='mdl_drag_coef_s',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=drag_coef_s)),
    Parameter(name='mdl_drag_coef_z',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=drag_coef_z)),
    Parameter(name='mdl_drag_partial_const_yaw',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                   double_value=drag_partial_const_yaw)),
]

# Pause between poses, good for stopping motion even if dynamics are poor,
# e.g., if drag or thrust parameters aren't very accurate. Must have PIDs on.
long_pause_auv_params = [
    Parameter(name='pose_plan_pause_duration',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.)),
]

# Don't pause between poses
no_pause_auv_params = [
    Parameter(name='pose_plan_pause_duration',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.)),
]

# Use ex.mission_info to distinguish between the motions
EXPERIMENT_F = 'drag experiment forward/back'
EXPERIMENT_S = 'drag experiment strafe left/right'
EXPERIMENT_Z = 'drag experiment vertical up/down'
EXPERIMENT_YAW = 'drag experiment yaw ccw/cw'


# Calculate drag coefficients
def process_messages(ex: MissionExperiment, logger):
    if len(ex.co_msgs) < 2:
        logger.info('too few control messages, wrong topic?')
        return

    logger.info('{} recorded {} control messages spanning {:.2f} seconds'.format(
        ex.mission_info, len(ex.co_msgs), seconds(
            ex.co_msgs[-1].header.stamp) - seconds(ex.co_msgs[0].header.stamp)))

    if ex.mission_info == EXPERIMENT_F:
        plan_run_v = auv_xy_velo
        plan_drag_coef = drag_coef_f
    elif ex.mission_info == EXPERIMENT_S:
        plan_run_v = auv_xy_velo
        plan_drag_coef = drag_coef_s
    elif ex.mission_info == EXPERIMENT_Z:
        plan_run_v = auv_z_velo
        plan_drag_coef = drag_coef_z
    else:
        plan_run_v = auv_yaw_velo
        plan_drag_coef = drag_coef_yaw

    msg: Control
    msg_start: Optional[Control] = None
    current_phase = MissionState.PHASE_NONE
    for msg in ex.co_msgs:
        if msg.mission.phase != current_phase:
            # Phase changed
            current_phase = msg.mission.phase

            if msg.mission.phase == MissionState.PHASE_TRAP_CONSTANT_V:
                # Starting run (constant velocity) phase
                msg_start = msg

            elif msg.mission.phase == MissionState.PHASE_TRAP_DECEL:
                # Starting deceleration phase, which means the run phase is over, or didn't happen
                if msg_start is None:
                    # There was no run phase
                    logger.info('no run phase, skipping segment')
                else:
                    # Run time
                    run_dt = seconds(msg.header.stamp) - seconds(msg_start.header.stamp)

                    # Run distance
                    if ex.mission_info == EXPERIMENT_F:
                        actual_run_d = abs(
                            msg.estimate.pose.pose.position.x -
                            msg_start.estimate.pose.pose.position.x)
                    elif ex.mission_info == EXPERIMENT_S:
                        actual_run_d = abs(
                            msg.estimate.pose.pose.position.y -
                            msg_start.estimate.pose.pose.position.y)
                    elif ex.mission_info == EXPERIMENT_Z:
                        actual_run_d = abs(
                            msg.estimate.pose.pose.position.z -
                            msg_start.estimate.pose.pose.position.z)
                    else:
                        actual_run_d = abs(norm_angle(
                            get_yaw(msg.estimate.pose.pose.orientation) -
                            get_yaw(msg_start.estimate.pose.pose.orientation)))

                    # Run velocity
                    actual_run_v = actual_run_d / run_dt

                    # The plan depends on the thrust force parameters and the drag coefficient
                    # parameters. If we assume that the thrust force parameters are accurate, we
                    # can use the difference between planned & actual velocity to calculate new
                    # drag coefficients.
                    actual_drag_coef = plan_drag_coef * pow(plan_run_v / actual_run_v, 2)

                    logger.info(('run phase dt {:.2f}, plan v {:.2f}, actual v {:.2f}, '
                           'actual d {:.3f}, plan drag {:.2f}, actual drag {:.2f}').format(
                        run_dt, plan_run_v, actual_run_v, actual_run_d, plan_drag_coef,
                        actual_drag_coef))

                    # Reset
                    msg_start = None


# Tweak these for ft3, other tests
base_z = -0.2  # x, y and yaw tests are run at this depth
extent_xy = 0.5  # x and y tests run from extent_xy to -extent_xy
high_z = -0.0  # Highest z point
low_z = -0.3  # Lowest z point
extent_yaw = 0.5  # yaw tests run from extent_yaw to -extent_yaw

ex_f_sequence = [
    # Move to start position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'move to start',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(x=extent_xy, z=base_z)),
        ],
        False,
        False,
        None
    ),
    # Experiment: PIDs off, no pause, back/forth, repeat motion several times
    MissionExperiment.go_to_poses(
        EXPERIMENT_F,
        1,
        straight_motion_auv_params + no_pids_auv_params + drag_auv_params + no_pause_auv_params,
        [],
        [
            Pose(position=Point(x=-extent_xy, z=base_z)),
            Pose(position=Point(x=extent_xy, z=base_z)),
        ],
        False,
        False,
        process_messages
    ),
    # Return to base position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'return to zero',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(x=extent_xy, z=base_z)),
        ],
        False,
        False,
        None
    ),
]

ex_s_sequence = [
    # Move to start position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'move to start',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(y=extent_xy, z=base_z)),
        ],
        False,
        False,
        None
    ),
    # Experiment: PIDs off, no pause, back/forth, repeat motion several times
    MissionExperiment.go_to_poses(
        EXPERIMENT_S,
        3,
        straight_motion_auv_params + no_pids_auv_params + drag_auv_params + no_pause_auv_params,
        [],
        [
            Pose(position=Point(y=-extent_xy, z=base_z)),
            Pose(position=Point(y=extent_xy, z=base_z)),
        ],
        False,
        False,
        process_messages
    ),
    # Return to base position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'return to zero',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(z=base_z)),
        ],
        False,
        False,
        None
    ),
]

ex_z_sequence = [
    # Move to start position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'move to start',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(z=high_z)),
        ],
        False,
        False,
        None
    ),
    # Experiment: PIDs off, no pause, back/forth
    MissionExperiment.go_to_poses(
        EXPERIMENT_Z,
        3,
        straight_motion_auv_params + no_pids_auv_params + drag_auv_params + no_pause_auv_params,
        [],
        [
            Pose(position=Point(z=low_z)),
            Pose(position=Point(z=high_z)),
        ],
        False,
        False,
        process_messages
    ),
    # Return to base position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'return to zero',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(z=base_z)),
        ],
        False,
        False,
        None
    ),
]

ex_yaw_sequence = [
    # Move to start position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'move to start',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(z=base_z), orientation=get_quaternion(extent_yaw)),
        ],
        False,
        False,
        None
    ),
    # Experiment: PIDs off, no pause, back/forth, repeat motion several times
    MissionExperiment.go_to_poses(
        EXPERIMENT_YAW,
        3,
        straight_motion_auv_params + no_pids_auv_params + drag_auv_params + no_pause_auv_params,
        [],
        [
            Pose(position=Point(z=base_z), orientation=get_quaternion(-extent_yaw)),
            Pose(position=Point(z=base_z), orientation=get_quaternion(extent_yaw)),
        ],
        False,
        False,
        process_messages
    ),
    # Return to base position: PIDs on, 5s pause
    MissionExperiment.go_to_poses(
        'return to zero',
        1,
        straight_motion_auv_params + yes_pids_auv_params + drag_auv_params + long_pause_auv_params,
        [],
        [
            Pose(position=Point(z=base_z)),
        ],
        False,
        False,
        None
    ),
]


def main(args=None):
    rclpy.init(args=args)

    node = MissionExperimentRunNode(ex_f_sequence + ex_f_sequence + ex_f_sequence)

    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.stop_mission_and_destroy_client()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
