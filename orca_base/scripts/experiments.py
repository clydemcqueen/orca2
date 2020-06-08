#!/usr/bin/env python3

"""
Run a series of experiments

Usage:

ros2 run orca_base experiments.py
"""

from typing import List, Optional

import numpy as np
import rclpy
import rclpy.logging
import transformations as xf
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from orca_msgs.action import Mission
from orca_msgs.msg import FiducialPoseStamped
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

import nees_fp


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def param_to_str(p: Parameter):
    val = '(type error)'
    if p.value.type == ParameterType.PARAMETER_BOOL:
        val = str(p.value.bool_value)
    elif p.value.type == ParameterType.PARAMETER_DOUBLE:
        val = str(p.value.double_value)
    elif p.value.type == ParameterType.PARAMETER_INTEGER:
        val = str(p.value.integer_value)
    return p.name + ': ' + val


# Experiment description
# TODO support all of the mission parameters
class Experiment(object):

    def __init__(self, note: str, count: int, base_params: List[Parameter], filter_params: List[Parameter]):
        # Description
        self.note = note

        # Parameters
        self.count = count
        self.base_params = base_params
        self.filter_params = filter_params

        # Results
        self.results = []

    def print(self):
        print('Experiment {}, mission={}, count={}'.format(self.note, 'random markers', self.count))
        for p in self.base_params:
            print('auv_node::{}'.format(param_to_str(p)))
        for p in self.filter_params:
            print('filter_node::{}'.format(param_to_str(p)))
        print('Results: {}'.format(self.results))
        print()


# Experiment runner
# The control flow is buried in the callbacks. Each experiment does 3 things:
# 1. sets parameters on auv_node (set_auv_node_params)
# 2. sets parameters on filter_node (set_filter_node_params)
# 3. start a mission (send_mission_goal)
# 4. repeat (start_next_experiment)
class RunNode(Node):

    def __init__(self):
        super().__init__('experiments')

        # Optionally record fp and gt, and calc NEES
        self._calc_nees = True
        if self._calc_nees:
            print('recording fp and ground truth, will calc NEES')

        # Optionally support filter_node
        self._filter_node_running = False

        # Set up experiments
        self._experiments = [
            # Experiment(note='simple controller', mode=Control.AUV_MARKER_SEQUENCE, count=1, base_params=[
            #     Parameter(name='planner_z_target',
            #               value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-1.5)),
            #     Parameter(name='auv_controller',
            #               value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
            # ], filter_params=[
            #     Parameter(name='four_dof',
            #               value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
            # ]),

            Experiment(note='random markers', count=10, base_params=[], filter_params=[]),

            # Experiment(note='six DoF filter', mode=Control.AUV_MARKER_SEQUENCE, count=10, base_params=[
            #     Parameter(name='planner_z_target',
            #               value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-1.5))
            # ], filter_params=[
            #     Parameter(name='four_dof',
            #               value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
            # ]),
            #
            # Experiment(note='four DoF filter', mode=Control.AUV_MARKER_SEQUENCE, count=10, base_params=[
            #     Parameter(name='planner_z_target',
            #               value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-1.5))
            # ], filter_params=[
            #     Parameter(name='four_dof',
            #               value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
            # ]),
        ]

        # If we're in a mission record fiducial pose and ground truth messages for later processing
        self._fp_msgs: Optional[List[FiducialPoseStamped]] = None
        self._gt_msgs: Optional[List[Odometry]] = None
        self._fp_sub = self.create_subscription(FiducialPoseStamped, '/filtered_fp', self.fp_cb, 10)
        self._gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        # Set parameter service clients
        self._set_param_auv_node_client = self.create_client(SetParameters, '/auv_node/set_parameters')
        self._set_param_filter_node_client = self.create_client(SetParameters, '/filter_node/set_parameters')

        # Mission action client
        self._mission_action_client = ActionClient(self, Mission, '/mission')

        # Futures manage the async state changes
        self._set_auv_node_params_future = None
        self._set_filter_node_params_future = None
        self._send_goal_future = None
        self._get_result_future = None

        # Active mission
        self._goal_handle = None

        # Start 1st experiment
        self._idx = 0
        self._count = 0
        self.get_logger().info('starting experiment {}, ({}), will run {} time(s)'.format(
            self._idx, self._experiments[self._idx].note, self._experiments[self._idx].count))
        self.get_logger().info('starting run {} of {}'.format(self._count + 1, self._experiments[self._idx].count))

        # Step 1
        self.set_auv_node_params()

    def fp_cb(self, msg: FiducialPoseStamped):
        if self._fp_msgs is not None:
            self._fp_msgs.append(msg)

    def gt_cb(self, msg: Odometry):
        if self._gt_msgs is not None:
            self._gt_msgs.append(msg)

    def set_auv_node_params(self):
        self.get_logger().debug('waiting for /auv_node/set_parameters server...')
        self._set_param_auv_node_client.wait_for_service()

        request = SetParameters.Request()
        for param in self._experiments[self._idx].base_params:
            request.parameters.append(param)

        self.get_logger().debug('setting auv_node params...')
        self._set_auv_node_params_future = self._set_param_auv_node_client.call_async(request)
        self._set_auv_node_params_future.add_done_callback(self.set_auv_node_params_done_cb)

    def set_auv_node_params_done_cb(self, _):
        self.get_logger().debug('auv_node params set')

        # Step 2
        self.set_filter_node_params()

    def set_filter_node_params(self):
        if self._filter_node_running:

            self.get_logger().debug('waiting for /filter_node/set_parameters server...')
            self._set_param_filter_node_client.wait_for_service()

            request = SetParameters.Request()
            for param in self._experiments[self._idx].filter_params:
                request.parameters.append(param)

            self.get_logger().debug('setting filter_node params...')
            self._set_filter_node_params_future = self._set_param_filter_node_client.call_async(request)
            self._set_filter_node_params_future.add_done_callback(self.set_filter_node_params_done_cb)

        else:

            self.send_mission_goal()

    def set_filter_node_params_done_cb(self, _):
        self.get_logger().debug('filter_node params set')

        # Step 3
        self.send_mission_goal()

    def send_mission_goal(self):
        self.get_logger().debug('waiting for /mission server...')
        self._mission_action_client.wait_for_server()

        self.get_logger().debug('sending goal request...')
        goal_msg = Mission.Goal()
        goal_msg.random = True  # Random markers
        self._send_goal_future = self._mission_action_client.send_goal_async(goal_msg,
                                                                             feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('goal accepted')

            # Save client goal handle, useful if we need to abort the mission
            self._goal_handle = goal_handle

            # Start recording messages
            if self._calc_nees:
                self._fp_msgs = []
                self._gt_msgs = []

            # Get notified when the mission is complete
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_cb)
        else:
            self.get_logger().warn('goal rejected')

            # Step 4
            self.start_next_experiment()

    def feedback_cb(self, feedback):
        self.get_logger().debug(
            'feedback: {0} out of {1}'.format(feedback.feedback.targets_completed, feedback.feedback.targets_total))

    def get_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.get_logger().info(
                'goal succeeded, result: {0} out of {1}'.format(result.targets_completed, result.targets_total))

            # Calc and report NEES
            # TODO sub drifts too much during this calc, move calc to sep thread
            if self._calc_nees:
                self.report_nees()

            # Stop recording messages
            self._fp_msgs = None
            self._gt_msgs = None
        else:
            self.get_logger().warn('goal failed with status: {0}'.format(status))

        # Clear goal_handle
        self._goal_handle = None

        # Step 4
        self.start_next_experiment()

    def report_nees(self):
        if self._fp_msgs is None or len(self._fp_msgs) < 5:
            self.get_logger().error('too few fp messages')
            return

        self.get_logger().info('{} fp messages spanning {} seconds'.format(
            len(self._fp_msgs),
            seconds(self._fp_msgs[-1].header.stamp) - seconds(self._fp_msgs[0].header.stamp)))

        if self._gt_msgs is None or len(self._gt_msgs) < 5:
            self.get_logger().error('too few ground truth messages')
            return

        self.get_logger().info('{} ground truth messages spanning {} seconds'.format(
            len(self._gt_msgs),
            seconds(self._gt_msgs[-1].header.stamp) - seconds(self._gt_msgs[0].header.stamp)))

        nees_values = nees_fp.nees(self._fp_msgs, self._gt_msgs)

        if nees_values:
            self._experiments[self._idx].results.append(np.mean(nees_values))
            self.get_logger().info('average NEES value {}'.format(self._experiments[self._idx].results[-1]))
        else:
            self._experiments[self._idx].results.append(-1)
            self.get_logger().info('no NEES values')

    def start_next_experiment(self):
        # Next run of this experiment
        self._count += 1
        if self._count < self._experiments[self._idx].count:
            self.get_logger().info('starting run {} of {}'.format(self._count + 1, self._experiments[self._idx].count))
            self.set_auv_node_params()
        else:
            # Next experiment
            self._idx += 1
            self._count = 0
            if self._idx < len(self._experiments):
                self.get_logger().info('starting experiment {}, ({}), will run {} time(s)'.format(
                    self._idx, self._experiments[self._idx].note, self._experiments[self._idx].count))
                self.get_logger().info('starting run {} of {}'.format(self._count + 1,
                                                                      self._experiments[self._idx].count))
                self.set_auv_node_params()
            else:
                self.get_logger().info('DONE!')
                self.print_results()

    def print_results(self):
        for experiment in self._experiments:
            experiment.print()

    def stop_mission_and_destroy_client(self):
        if self._goal_handle is not None:
            print('stop mission')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        if self._mission_action_client is not None:
            print('destroy action client')
            # Avoids a crash in rclpy/action/client.py
            self._mission_action_client.destroy()
            self._mission_action_client = None


def main(args=None):
    rclpy.init(args=args)

    node = RunNode()

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
