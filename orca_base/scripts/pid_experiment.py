#!/usr/bin/env python3

"""
Run experiments to calculate PID coefficients.

WIP... working on an auto PID tuner

Usage:
ros2 run orca_base pid_experiment.py
"""

from typing import Dict, Optional
import rclpy
import rclpy.logging
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from mission_experiment import MissionExperiment, MissionExperimentRunNode
from geometry_msgs.msg import Point, Pose


# See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
def zn_classic_pid(Ku, Tu):
    return [0.6 * Ku, 1.2 * Ku / Tu, 0.075 * Ku * Tu]


def zn_pessen(Ku, Tu):
    return [0.7 * Ku, 1.75 * Ku / Tu, 0.105 * Ku * Tu]


def zn_no_overshoot(Ku, Tu):
    return [0.2 * Ku, 0.4 * Ku / Tu, 0.0667 * Ku * Tu]


def pid_param_dict(dim: str, values):
    names = ['auv_' + dim + '_pid_kp', 'auv_' + dim + '_pid_ki', 'auv_' + dim + '_pid_kd']
    return {name: value for name, value in zip(names, values)}


def dict_to_obj(d: Dict):
    return [Parameter(name=item[0], value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=item[1]))
            for item in d.items()]


def pid_param_obj(dim: str, values):
    d = pid_param_dict(dim, values)
    print(d)
    return dict_to_obj(d)


def default_pid_params(dim: str):
    return pid_param_obj(dim, [0.5, 0.0, 0.0])


def zn_no_overshoot_params(dim: str, Ku, Tu):
    return pid_param_obj(dim, zn_no_overshoot(Ku, Tu))


base_auv_params = [
    # Never replan
    Parameter(name='global_plan_max_xy_err',
                         value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0))
]

def_x_pids = default_pid_params('x')
good_x_pids = zn_no_overshoot_params('x', 10.0, 2.05)
def_y_pids = default_pid_params('y')
test_y_pids = pid_param_obj('y', [15.0, 0.0, 0.0])
good_y_pids = zn_no_overshoot_params('y', 15.0, 2.4)
def_z_pids = default_pid_params('z')
def_yaw_pids = default_pid_params('yaw')

all_auv_params = base_auv_params + good_x_pids + good_y_pids + def_z_pids + def_yaw_pids

# Hold at one spot
ex_hold = MissionExperiment.go_to_poses(
    'move to start',
    1,
    all_auv_params,
    [],
    [
        Pose(position=Point(x=0.0, z=-0.5)),
    ],
    False,
    True,  # Hold position
    None
)

# Run around
ex_run = MissionExperiment.go_to_markers(
    'visit all markers in a random order',
    1,
    all_auv_params,
    [],
    [],  # No markers == all markers
    True,  # Visit markers in random order
    False,
    None
)


def main(args=None):
    rclpy.init(args=args)

    node = MissionExperimentRunNode([ex_hold])

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
