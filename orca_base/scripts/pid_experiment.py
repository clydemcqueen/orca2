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
Auto P tuner.

Runs experiments to find good x, y, z and yaw kp values.

Usage:
ros2 launch orca_gazebo sim_launch.py  # use World.SMALL_FIELD
ros2 run orca_base pid_experiment.py
"""

from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point, Pose
from mission_experiment import MissionExperiment, MissionExperimentRunNode
from orca_util import get_quaternion, get_yaw, norm_angle, seconds
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import rclpy
import rclpy.logging

# kp is increased until P controller causes value to hit the target in < TIME_TO_CORRECT seconds.
# To get more aggressive P controllers:
# -- reduce TIME_TO_CORRECT
# -- increase distance from ex_start to ex_jump
TIME_TO_CORRECT = 4.0


# Get a parameter value
def get_param_double(params: List[Parameter], name: str) -> Optional[float]:
    for param in params:
        if param.name == name:
            return param.value.double_value
    return None


# Set a parameter value
def set_param_double(params: List[Parameter], name: str, value: float):
    for param in params:
        if param.name == name:
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            return
    print('ERROR parameter', name, 'is not in the list')


# Adjust Kp for the next experiment
def adjust_kp(kp: float, target_value: float, target_time_to_correct: float, angle: bool,
              times: List[float], values: List[float]) -> Tuple[float, bool]:
    kp_inc = 0.2
    start_time = times[0]
    print('target value', target_value)
    # print(values)
    for time, value in zip(times[1:], values[1:]):
        diff = abs(norm_angle(value - target_value)) if angle else abs(value - target_value)
        if diff < 0.01:
            time_to_correct = time - start_time
            print('time_to_correct', time_to_correct, 'seconds')
            if time_to_correct > target_time_to_correct:
                kp += kp_inc
                return kp, False  # Continue
            else:
                return kp, True  # Stop

    # Never hit target
    print('never hit target')
    kp += kp_inc
    return kp, False  # Continue


# Process messages, return True to stop
def process_messages(ex: MissionExperiment):
    if len(ex.co_msgs) < 2:
        print('too few control messages, wrong topic?')
        return

    print('{} recorded {} control messages spanning {:.2f} seconds'.format(
        ex.mission_info, len(ex.co_msgs), seconds(
            ex.co_msgs[-1].header.stamp) - seconds(ex.co_msgs[0].header.stamp)))

    x_kp = get_param_double(ex.auv_params, 'auv_x_pid_kp')
    y_kp = get_param_double(ex.auv_params, 'auv_y_pid_kp')
    z_kp = get_param_double(ex.auv_params, 'auv_z_pid_kp')
    yaw_kp = get_param_double(ex.auv_params, 'auv_yaw_pid_kp')

    times = [seconds(msg.header.stamp) for msg in ex.co_msgs]
    x_values = [msg.estimate.pose.pose.position.x for msg in ex.co_msgs]
    y_values = [msg.estimate.pose.pose.position.y for msg in ex.co_msgs]
    z_values = [msg.estimate.pose.pose.position.z for msg in ex.co_msgs]
    yaw_values = [get_yaw(msg.estimate.pose.pose.orientation) for msg in ex.co_msgs]

    # co_msgs[0].mission.pose.fp.pose.pose is (0, 0, 0) (why?), select co_msgs[1]
    x_kp, x_stop = adjust_kp(x_kp, ex.co_msgs[1].mission.pose.fp.pose.pose.position.x,
                             TIME_TO_CORRECT, False, times, x_values)

    if x_stop:
        print('x_kp is good at', x_kp)
    else:
        print('bump x_kp to', x_kp)
        set_param_double(ex.auv_params, 'auv_x_pid_kp', x_kp)

    y_kp, y_stop = adjust_kp(y_kp, ex.co_msgs[1].mission.pose.fp.pose.pose.position.y,
                             TIME_TO_CORRECT, False, times, y_values)

    if y_stop:
        print('y_kp is good at', y_kp)
    else:
        print('bump y_kp to', y_kp)
        set_param_double(ex.auv_params, 'auv_y_pid_kp', y_kp)

    z_kp, z_stop = adjust_kp(z_kp, ex.co_msgs[1].mission.pose.fp.pose.pose.position.z,
                             TIME_TO_CORRECT, False, times, z_values)

    if z_stop:
        print('z_kp is good at', z_kp)
    else:
        print('bump z_kp to', z_kp)
        set_param_double(ex.auv_params, 'auv_z_pid_kp', z_kp)

    yaw_kp, yaw_stop = adjust_kp(yaw_kp,
                                 get_yaw(ex.co_msgs[1].mission.pose.fp.pose.pose.orientation),
                                 TIME_TO_CORRECT, True, times, yaw_values)

    if yaw_stop:
        print('yaw_kp is good at', yaw_kp)
    else:
        print('bump yaw_kp to', yaw_kp)
        set_param_double(ex.auv_params, 'auv_yaw_pid_kp', yaw_kp)

    return x_stop and y_stop and z_stop and yaw_stop


# Generate a param dict for PID coefficients
def pid_param_dict(dim: str, values):
    names = ['auv_' + dim + '_pid_kp', 'auv_' + dim + '_pid_ki', 'auv_' + dim + '_pid_kd']
    return {name: value for name, value in zip(names, values)}


# Turn a param dict into a list of rclpy Parameter objects.
# Works for ParameterType.PARAMETER_DOUBLE.
def dict_to_obj(d: Dict):
    return [Parameter(name=item[0],
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                           double_value=item[1]))
            for item in d.items()]


# Given a list of parameter names & values,
# ... print a param dict
# ... return a list of rclpy Parameter objects
def pid_param_obj(dim: str, values):
    d = pid_param_dict(dim, values)
    print(d)
    return dict_to_obj(d)


# Get default PID parameters
def default_pid_params(dim: str):
    return pid_param_obj(dim, [0.5, 0.0, 0.0])


# Get zn_no_overshoot PID parameters
# Not used in ft3
def zn_no_overshoot(Ku, Tu):
    return [0.2 * Ku, 0.4 * Ku / Tu, 0.0667 * Ku * Tu]


def zn_no_overshoot_params(dim: str, Ku, Tu):
    return pid_param_obj(dim, zn_no_overshoot(Ku, Tu))


base_auv_params = [
    # Never replan
    Parameter(name='global_plan_max_xy_err',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0)),

    # Trust markers up to 3m away
    Parameter(name='good_pose_dist',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=3.0)),
]

# Normal motion, with local planning
# Use the planned yaw to rotate body to world frame (this is the default)
normal_motion_params = [
    Parameter(name='pose_plan_pause_duration',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)),
    Parameter(name='pose_plan_epsilon_xyz',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.05)),
    Parameter(name='pose_plan_epsilon_yaw',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.05)),
    Parameter(name='pose_plan_max_short_plan_xy',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0)),
    Parameter(name='control_use_est_yaw',
              value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)),
]

# PID-only motion, no local planning
# Use the estimated yaw to rotate body to world frame -- markers must be in view at all times!
pid_only_motion_params = [
    Parameter(name='pose_plan_pause_duration',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0)),
    Parameter(name='pose_plan_epsilon_xyz',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10000.0)),
    Parameter(name='pose_plan_epsilon_yaw',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10000.0)),
    Parameter(name='pose_plan_max_short_plan_xy',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0)),
    Parameter(name='control_use_est_yaw',
              value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
]

def_x_pids = default_pid_params('x')
# good_x_pids = zn_no_overshoot_params('x', 10.0, 2.05)
def_y_pids = default_pid_params('y')
# test_y_pids = pid_param_obj('y', [15.0, 0.0, 0.0])
# good_y_pids = zn_no_overshoot_params('y', 15.0, 2.4)
def_z_pids = default_pid_params('z')
def_yaw_pids = default_pid_params('yaw')

all_pid_params = def_x_pids + def_y_pids + def_z_pids + def_yaw_pids

# An 'experiment' consists of a set of AUV params, filter params, and 1 or more targets.
# We string experiments together to set different parameters:
# -- ex_start moves to the start position using normal planning
# -- ex_jump moves to the end position using just the P controller
# This is repeated until the kp values stop changing.

# Move to start position
ex_start = MissionExperiment.go_to_poses(
    'move to start',
    1,
    base_auv_params + normal_motion_params + all_pid_params,
    [],
    [
        Pose(position=Point(x=-0.25, y=-0.25, z=-0.75), orientation=get_quaternion(-0.5)),
    ],
    False,
    False,  # Do not hold position
    None
)

# Use PID-only motion to move to pose2
ex_jump = MissionExperiment.go_to_poses(
    'jump',
    1,
    base_auv_params + pid_only_motion_params + all_pid_params,
    [],
    [
        Pose(position=Point(x=0.25, y=0.25, z=-0.25), orientation=get_quaternion(0.5)),
    ],
    False,
    False,  # Do not hold position
    process_messages
)

# Run around
ex_simple_test = MissionExperiment.go_to_markers(
    'visit all markers in a random order',
    1,
    base_auv_params + normal_motion_params + all_pid_params,
    [],
    [],  # No markers == all markers
    True,  # Visit markers in random order
    False,  # Do not hold position
    None
)


def main(args=None):
    rclpy.init(args=args)

    # Run experiments to find good kp values for x, y, z, yaw
    node = MissionExperimentRunNode([ex_start, ex_jump], repeat=True)

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
