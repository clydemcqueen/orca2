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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from orca_msgs.action import Mission
from orca_msgs.msg import FiducialPoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from orca_util import get_quaternion, seconds
import nees_fp


def param_to_str(p: Parameter):
    val = '(type error)'
    if p.value.type == ParameterType.PARAMETER_BOOL:
        val = str(p.value.bool_value)
    elif p.value.type == ParameterType.PARAMETER_DOUBLE:
        val = str(p.value.double_value)
    elif p.value.type == ParameterType.PARAMETER_INTEGER:
        val = str(p.value.integer_value)
    return p.name + ': ' + val


class Experiment(object):
    """
    Experiment description
    """

    def __init__(self, name: str, count: int, auv_params: List[Parameter], filter_params: List[Parameter],
                 pose_targets: bool, marker_ids: List[int], poses: List[Pose], random: bool):
        # Description
        self.name = name

        # Number of times to repeat the experiment
        self.count = count

        # Parameters
        self.auv_params = auv_params
        self.filter_params = filter_params

        # Marker mission or pose mission
        self.pose_targets = pose_targets

        # If !pose_targets, then a list of marker ids
        self.marker_ids = marker_ids

        # If pose_targets, then a list of poses
        self.poses = poses

        # Randomize the markers or poses
        self.random = random

        # Results
        self.results = []

    def log_info(self, logger):
        logger.info('experiment name={}, count={}, pose_targets={}'.format(self.name, self.count, self.pose_targets))

        if self.pose_targets:
            for p in self.poses:
                logger.info('pose x={}, y={}, z={}'.format(p.position.x, p.position.y, p.position.z))
        else:
            logger.info('marker_ids={}'.format(self.marker_ids))

        for p in self.auv_params:
            logger.info('auv_node.{}'.format(param_to_str(p)))

        for p in self.filter_params:
            logger.info('filter_node.{}'.format(param_to_str(p)))

    def create_goal(self):
        goal_msg = Mission.Goal()
        goal_msg.pose_targets = self.pose_targets
        goal_msg.marker_ids = self.marker_ids
        goal_msg.poses = self.poses
        goal_msg.random = self.random
        return goal_msg

    @classmethod
    def go_to_markers(cls, note: str, count: int, auv_params: List[Parameter], filter_params: List[Parameter],
                      marker_ids: List[int], random: bool):
        return cls(note, count, auv_params, filter_params, False, marker_ids, [], random)

    @classmethod
    def go_to_poses(cls, note: str, count: int, auv_params: List[Parameter], filter_params: List[Parameter],
                    poses: List[Pose], random: bool):
        return cls(note, count, auv_params, filter_params, True, [], poses, random)


class RunNode(Node):
    """
    Experiment runner

    The control flow is buried in the callbacks. Each experiment does 3 things:
    1. sets parameters on auv_node (set_auv_node_params)
    2. sets parameters on filter_node (set_filter_node_params)
    3. start a mission (send_mission_goal)
    4. repeat (start_next_experiment)
    """

    def __init__(self, experiments, filter_node_running, calc_nees):
        super().__init__('experiment_runner')

        assert len(experiments) > 0
        self._experiments = experiments

        # NEES is designed for ground-truth experiments with a filter
        assert filter_node_running or not calc_nees

        # Optionally support filter_node
        self._filter_node_running = filter_node_running
        if self._filter_node_running:
            self.get_logger().info('filter node running')
        else:
            self.get_logger().info('no filter node')

        # Optionally record fp and gt, and calc NEES
        self._calc_nees = calc_nees
        if self._calc_nees:
            self.get_logger().info('recording fp and ground truth, will calc NEES')
        else:
            self.get_logger().info('will not calc NEES')

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
        self.start_experiment()

    def start_experiment(self):
        """Start experiment self._idx"""
        self.get_logger().info('starting experiment {}'.format(self._idx))
        self._experiments[self._idx].log_info(self.get_logger())
        self._count = 0
        self.start_run()

    def start_run(self):
        """Start run self._count in experiment self._idx"""
        self.get_logger().info('starting run {} of {}'.format(self._count + 1, self._experiments[self._idx].count))

        # Step 1
        self.set_auv_node_params()

    def start_next_run_or_experiment_or_done(self):
        """Next run of experiment self_.idx, or next experiment, or we're done"""
        self._count += 1
        if self._count < self._experiments[self._idx].count:
            self.start_run()
        else:
            # Next experiment
            self._idx += 1
            self._count = 0
            if self._idx < len(self._experiments):
                self.start_experiment()
            else:
                self.get_logger().info('DONE!')

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
        for param in self._experiments[self._idx].auv_params:
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
        self._send_goal_future = self._mission_action_client.send_goal_async(self._experiments[self._idx].create_goal(),
                                                                             feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('goal accepted')

            # Save client goal handle, useful if we need to abort the mission
            self._goal_handle = goal_handle

            # Start recording messages
            # TODO move to Experiment class
            if self._calc_nees:
                self._fp_msgs = []
                self._gt_msgs = []

            # Get notified when the mission is complete
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_cb)
        else:
            self.get_logger().warn('goal rejected')

            # Step 4
            self.start_next_run_or_experiment_or_done()

    def feedback_cb(self, feedback):
        self.get_logger().debug(
            'feedback: {0} out of {1}'.format(feedback.feedback.targets_completed, feedback.feedback.targets_total))

    def get_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.get_logger().info(
                'goal succeeded, completed {0} out of {1}'.format(result.targets_completed, result.targets_total))

            # Calc and report NEES
            # Note: this takes several seconds -- way too long for a callback
            # The sub is drifting while we're waiting, which often means that subsequent experiments fail
            # TODO move to Experiment class, and run on a different thread
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
        self.start_next_run_or_experiment_or_done()

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


# Some experiments, edit this file as needed

# Nice smoke test: run through all markers in random order, 10 times
# Use this w/ the default parameters
random_markers_experiment = Experiment.go_to_markers('random markers', 10, [], [], [], True)

# Use these parameters to force a simple back/forth motion
# Note that PIDs are still active
straight_motion_auv_params = [
    # pose_plan_max_short_plan_xy controls how much distance to cover with an "all DoF" motion,
    # vs. the more typical "long plan" that turns torward the motion, runs, then turns again.
    # Set this to a large value so that we can run backwards and side-to-side.
    Parameter(name='pose_plan_max_short_plan_xy',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=99.)),
    # good_pose_dist controls how close we need to be to a marker to trust the pose.
    # Set this to a large-ish value so that we always build a local plan at each pose.
    Parameter(name='good_pose_dist',
              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=4.)),
]

# Turn off PID controllers
no_pids_auv_params = [
    Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.))
    for name in ['auv_x_pid_kp', 'auv_x_pid_ki', 'auv_x_pid_kd',
                 'auv_y_pid_kp', 'auv_y_pid_ki', 'auv_y_pid_kd',
                 'auv_z_pid_kp', 'auv_z_pid_ki', 'auv_z_pid_kd',
                 'auv_yaw_pid_kp', 'auv_yaw_pid_ki', 'auv_yaw_pid_kd']
]

# Move back/forth in x
move_x_experiment = Experiment.go_to_poses(
    'move_x',
    3,
    straight_motion_auv_params + no_pids_auv_params,
    [],
    [
        Pose(position=Point(x=0.5, z=-0.5)),
        Pose(position=Point(x=-0.5, z=-0.5))
    ],
    False
)

# Move back/forth in y
move_y_experiment = Experiment.go_to_poses(
    'move_y',
    10,
    straight_motion_auv_params + no_pids_auv_params,
    [],
    [
        Pose(position=Point(y=0.5, z=-0.5)),
        Pose(position=Point(y=-0.5, z=-0.5))
    ],
    False
)

# Move back/forth in z
move_z_experiment = Experiment.go_to_poses(
    'move_z',
    3,
    straight_motion_auv_params + no_pids_auv_params,
    [],
    [
        Pose(position=Point(z=-0.1)),
        Pose(position=Point(z=-0.9))
    ],
    False
)

# Rotate back/forth
rotate_experiment = Experiment.go_to_poses(
    'rotate',
    3,
    straight_motion_auv_params + no_pids_auv_params,
    [],
    [
        Pose(position=Point(z=-0.5), orientation=get_quaternion(1.5)),
        Pose(position=Point(z=-0.5), orientation=get_quaternion(-1.5))
    ],
    False
)


def main(args=None):
    rclpy.init(args=args)

    node = RunNode([move_y_experiment], False, False)

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
