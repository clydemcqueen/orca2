#!/usr/bin/env python3

"""
Run a series of experiments

Usage: ros2 run orca_base experiments.py
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
from orca_msgs.msg import Control
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

import nees


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


# Experiment description
class Experiment(object):

    def __init__(self, note: str, mode: int, base_params: List[Parameter], filter_params: List[Parameter]):
        self.note = note
        self.mode = mode
        self.base_params = base_params
        self.filter_params = filter_params


# Experiment runner
# The control flow is buried in the callbacks. Each experiment does 3 things:
# 1. sets parameters on base_node (set_base_node_params)
# 2. sets parameters on filter_node (set_filter_node_params)
# 3. start a mission (send_mission_goal)
# 4. repeat (start_next_experiment)
class RunNode(Node):

    def __init__(self):
        super().__init__('experiments')

        # Set up experiments
        self._experiments = [
            Experiment(note='warm up run', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
            ]),

            Experiment(note='six DoF filter 1', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
            ]),

            Experiment(note='four DoF filter 1', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
            ]),

            Experiment(note='six DoF filter 2', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
            ]),

            Experiment(note='four DoF filter 2', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
            ]),

            Experiment(note='six DoF filter 3', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
            ]),

            Experiment(note='four DoF filter 3', mode=Control.AUV_SEQUENCE, base_params=[], filter_params=[
                Parameter(name='four_dof',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
            ]),

        ]

        # If we're in a mission record odometry and ground truth messages for later processing
        self._odom_msgs: Optional[List[Odometry]] = None
        self._gt_msgs: Optional[List[Odometry]] = None
        self._odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 5)
        self._gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 5)

        # Set parameter service clients
        self._set_param_base_node_client = self.create_client(SetParameters, '/base_node/set_parameters')
        self._set_param_filter_node_client = self.create_client(SetParameters, '/filter_node/set_parameters')

        # Mission action client
        self._mission_action_client = ActionClient(self, Mission, '/mission')

        # Futures manage the async state changes
        self._set_base_node_params_future = None
        self._set_filter_node_params_future = None
        self._send_goal_future = None
        self._get_result_future = None

        # Start 1st experiment
        self._idx = 0
        self.get_logger().info('starting experiment {}, "{}"'.format(self._idx, self._experiments[self._idx].note))

        # Step 1
        self.set_base_node_params()

    def odom_cb(self, msg: Odometry):
        if self._odom_msgs is not None:
            self._odom_msgs.append(msg)

    def gt_cb(self, msg: Odometry):
        if self._gt_msgs is not None:
            self._gt_msgs.append(msg)

    def set_base_node_params(self):
        self.get_logger().debug('waiting for /base_node/set_parameters server...')
        self._set_param_base_node_client.wait_for_service()

        request = SetParameters.Request()
        for param in self._experiments[self._idx].base_params:
            request.parameters.append(param)

        self.get_logger().debug('setting base_node params...')
        self._set_base_node_params_future = self._set_param_base_node_client.call_async(request)
        self._set_base_node_params_future.add_done_callback(self.set_base_node_params_done_cb)

    def set_base_node_params_done_cb(self, _):
        self.get_logger().debug('base_node params set')

        # Step 2
        self.set_filter_node_params()

    def set_filter_node_params(self):
        self.get_logger().debug('waiting for /filter_node/set_parameters server...')
        self._set_param_filter_node_client.wait_for_service()

        request = SetParameters.Request()
        for param in self._experiments[self._idx].filter_params:
            request.parameters.append(param)

        self.get_logger().debug('setting filter_node params...')
        self._set_filter_node_params_future = self._set_param_filter_node_client.call_async(request)
        self._set_filter_node_params_future.add_done_callback(self.set_filter_node_params_done_cb)

    def set_filter_node_params_done_cb(self, _):
        self.get_logger().debug('filter_node params set')

        # Step 3
        self.send_mission_goal()

    def send_mission_goal(self):
        self.get_logger().debug('waiting for /mission server...')
        self._mission_action_client.wait_for_server()

        self.get_logger().debug('sending goal request...')
        goal_msg = Mission.Goal()
        goal_msg.mode = self._experiments[self._idx].mode
        self._send_goal_future = self._mission_action_client.send_goal_async(goal_msg,
                                                                             feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('goal accepted')

            # Start recording messages
            self._odom_msgs = []
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
            'feedback: {0} out of {1}'.format(feedback.feedback.segments_completed, feedback.feedback.segments_total))

    def get_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.get_logger().info(
                'goal succeeded, result: {0} out of {1}'.format(result.segments_completed, result.segments_total))

            # Calc and report NEES
            self.report_nees()

            # Stop recording messages
            self._odom_msgs = None
            self._gt_msgs = None
        else:
            self.get_logger().warn('goal failed with status: {0}'.format(status))

        # Step 4
        self.start_next_experiment()

    def report_nees(self):
        if self._odom_msgs is None or len(self._odom_msgs) < 5:
            self.get_logger().error('too few odom messages')
            return

        self.get_logger().info('{} odom messages spanning {} seconds'.format(
            len(self._odom_msgs),
            seconds(self._odom_msgs[-1].header.stamp) - seconds(self._odom_msgs[0].header.stamp)))

        if self._gt_msgs is None or len(self._gt_msgs) < 5:
            self.get_logger().error('too few ground truth messages')
            return

        self.get_logger().info('{} ground truth messages spanning {} seconds'.format(
            len(self._gt_msgs),
            seconds(self._gt_msgs[-1].header.stamp) - seconds(self._gt_msgs[0].header.stamp)))

        nees_values = nees.nees(self._odom_msgs, self._gt_msgs)

        if nees_values:
            self.get_logger().info('average NEES value {}'.format(np.mean(nees_values)))
        else:
            self.get_logger().info('no NEES values')

    def start_next_experiment(self):
        self._idx += 1
        if self._idx < len(self._experiments):
            self.get_logger().info('starting experiment {}, "{}"'.format(self._idx, self._experiments[self._idx].note))
            self.set_base_node_params()
        else:
            self.get_logger().info('DONE!')


def main(args=None):
    rclpy.init(args=args)

    node = RunNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
