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

from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from orca_msgs.action import Mission
from orca_msgs.msg import Control, FiducialPoseStamped, PoseBody
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node


def param_to_str(p: Parameter):
    val = '(type error)'
    if p.value.type == ParameterType.PARAMETER_BOOL:
        val = str(p.value.bool_value)
    elif p.value.type == ParameterType.PARAMETER_DOUBLE:
        val = str(p.value.double_value)
    elif p.value.type == ParameterType.PARAMETER_INTEGER:
        val = str(p.value.integer_value)
    return p.name + ': ' + val


# Future: class ParameterManager(object)
# __init__() gets all existing parameter values
# change() sends changes
# restore() restores existing values


class MissionExperiment(object):
    """Description of a mission experiment."""

    def __init__(self,
                 mission_info: str,
                 count: int,
                 auv_params: List[Parameter],
                 filter_params: List[Parameter],
                 target_type: int,
                 marker_ids: List[int],
                 poses: List[Pose],
                 motions: List[PoseBody],
                 random: bool,
                 keep_station: bool,
                 msg_processor):
        # Mission info string
        self.mission_info = mission_info

        # Number of times to repeat the mission
        self.count = count

        # Parameters
        self.auv_params = auv_params
        self.filter_params = filter_params

        # Marker mission or pose mission?
        self.target_type = target_type

        # If target_type is markers, then a list of marker ids
        self.marker_ids = marker_ids

        # If target_type is poses, then a list of poses
        self.poses = poses

        # If target_type is motions, then a list of motions
        self.motions = motions

        # Randomize the markers or poses
        self.random = random

        # Keep station at the last target
        self.keep_station = keep_station

        # Log several messages
        self.co_msgs: Optional[List[Control]] = None
        self.fp_msgs: Optional[List[FiducialPoseStamped]] = None
        self.gt_msgs: Optional[List[Odometry]] = None

        # Message processor
        self.msg_processor = msg_processor
        if msg_processor:
            self.co_msgs = []
            self.fp_msgs = []
            self.gt_msgs = []

    def log_info(self, logger):
        logger.info(
            'experiment mission_info={}, count={}, target_type={}'.format(self.mission_info,
                                                                           self.count,
                                                                           self.target_type))

        if self.target_type == Mission.Goal.TARGET_POSE:
            for p in self.poses:
                logger.info(
                    'pose x={}, y={}, z={}'.format(p.position.x, p.position.y, p.position.z))
        elif self.target_type == Mission.Goal.TARGET_MARKER:
            logger.info('marker_ids={}'.format(self.marker_ids))
        else:
            logger.error('NOT IMPLEMENTED YET')

        for p in self.auv_params:
            logger.info('auv_node.{}'.format(param_to_str(p)))

        for p in self.filter_params:
            logger.info('filter_node.{}'.format(param_to_str(p)))

    def get_goal_msg(self):
        goal_msg = Mission.Goal()
        goal_msg.mission_info = self.mission_info
        goal_msg.target_type = self.target_type
        goal_msg.marker_ids = self.marker_ids
        goal_msg.poses = self.poses
        goal_msg.motions = self.motions
        goal_msg.random = self.random
        goal_msg.keep_station = self.keep_station
        return goal_msg

    # Can call this at any time, but the general idea is to call this once when the experiment
    # is over. msg_processor should return True to stop experiments
    def process_messages(self) -> bool:
        if self.msg_processor:
            result = self.msg_processor(self)
            self.co_msgs.clear()
            self.fp_msgs.clear()
            self.gt_msgs.clear()
            return result
        else:
            print('no message processor')
            return False  # Continue

    @classmethod
    def go_to_markers(cls, mission_info: str, count: int, auv_params: List[Parameter],
                      filter_params: List[Parameter],
                      marker_ids: List[int], random: bool, keep_station: bool, msg_processor=None):
        return cls(mission_info, count, auv_params, filter_params, False, marker_ids, [], [],
                   random, keep_station, msg_processor)

    @classmethod
    def go_to_poses(cls, mission_info: str, count: int, auv_params: List[Parameter],
                    filter_params: List[Parameter],
                    poses: List[Pose], random: bool, keep_station: bool, msg_processor=None):
        return cls(mission_info, count, auv_params, filter_params, True, [], poses, [], random,
                   keep_station, msg_processor)


class MissionExperimentRunNode(Node):
    """
    Run a sequence of mission experiments.

    The control flow is buried in the callbacks. Each experiment does 3 things:
    1. sets parameters on auv_node (set_auv_node_params)
    2. sets parameters on filter_node (set_filter_node_params)
    3. starts a mission (send_mission_goal)
    4. repeat (start_next_run_or_experiment_or_done)
    """

    def __init__(self, experiments, repeat: bool = False):
        super().__init__('experiment_runner')

        assert len(experiments) > 0
        self._experiments = experiments

        # Subscribe to several messages for later processing
        self._co_sub = self.create_subscription(Control, '/control', self.co_cb, 10)
        self._fp_sub = self.create_subscription(FiducialPoseStamped, '/filtered_fp',
                                                self.fp_cb, 10)
        self._gt_sub = self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)

        # Set parameter service clients
        self._set_param_auv_node_client = self.create_client(SetParameters,
                                                             '/auv_node/set_parameters')
        self._set_param_filter_node_client = self.create_client(SetParameters,
                                                                '/filter_node/set_parameters')

        # Mission action client
        self._mission_action_client = ActionClient(self, Mission, '/mission')

        # Futures manage the async state changes
        self._set_auv_node_params_future = None
        self._set_filter_node_params_future = None
        self._send_goal_future = None
        self._get_result_future = None

        # Active mission
        self._goal_handle = None

        # Repeat sequence of experiments forever, or until msg_processor returns True
        self._repeat = repeat

        # Keep track of the current experiment
        self._idx = 0
        self._count = 0

        # Start 1st experiment
        self.start_first_experiment()

    def start_first_experiment(self):
        self._idx = 0
        self.start_experiment()

    def start_experiment(self):
        """Start next experiment."""
        self.get_logger().info('starting experiment {}'.format(self._idx))
        self._experiments[self._idx].log_info(self.get_logger())
        self._count = 0
        self.start_run()

    def start_run(self):
        """Start next run."""
        self.get_logger().info(
            'starting run {} of {}'.format(self._count + 1, self._experiments[self._idx].count))

        # Step 1
        self.set_auv_node_params()

    def start_next_run_or_experiment_or_done(self):
        """Start next run, or next experiment, or we're done."""
        self._count += 1
        if self._count < self._experiments[self._idx].count:
            self.start_run()
        else:
            self._idx += 1
            self._count = 0
            if self._idx < len(self._experiments):
                self.start_experiment()
            elif self._repeat:
                self.get_logger().info('REPEAT')
                self.start_first_experiment()
            else:
                self.get_logger().info('DONE!')

    def co_cb(self, msg: Control):
        if 0 <= self._idx < len(self._experiments) and self._experiments[
                self._idx].co_msgs is not None:
            self._experiments[self._idx].co_msgs.append(msg)

    def fp_cb(self, msg: FiducialPoseStamped):
        if 0 <= self._idx < len(self._experiments) and self._experiments[
                self._idx].fp_msgs is not None:
            self._experiments[self._idx].fp_msgs.append(msg)

    def gt_cb(self, msg: Odometry):
        if 0 <= self._idx < len(self._experiments) and self._experiments[
                self._idx].gt_msgs is not None:
            self._experiments[self._idx].gt_msgs.append(msg)

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
        if len(self._experiments[self._idx].filter_params) > 0:

            self.get_logger().debug('waiting for /filter_node/set_parameters server...')
            self._set_param_filter_node_client.wait_for_service()

            request = SetParameters.Request()
            for param in self._experiments[self._idx].filter_params:
                request.parameters.append(param)

            self.get_logger().debug('setting filter_node params...')
            self._set_filter_node_params_future = self._set_param_filter_node_client.call_async(
                request)
            self._set_filter_node_params_future.add_done_callback(
                self.set_filter_node_params_done_cb)

        else:

            # Step 3
            self.send_mission_goal()

    def set_filter_node_params_done_cb(self, _):
        self.get_logger().debug('filter_node params set')

        # Step 3
        self.send_mission_goal()

    def send_mission_goal(self):
        self.get_logger().debug('waiting for /mission server...')
        self._mission_action_client.wait_for_server()

        self.get_logger().debug('sending goal request...')
        self._send_goal_future = self._mission_action_client.send_goal_async(
            self._experiments[self._idx].get_goal_msg(),
            feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('goal accepted')

            # Save client goal handle, useful if we need to abort the mission
            self._goal_handle = goal_handle

            # Get notified when the mission is complete
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_cb)
        else:
            self.get_logger().warn('goal rejected, STOPPING')

    def feedback_cb(self, feedback):
        self.get_logger().debug(
            'feedback: {0} out of {1}'.format(feedback.feedback.targets_completed,
                                              feedback.feedback.targets_total))

    def get_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.get_logger().info(
                'goal succeeded, completed {0} out of {1}'.format(result.targets_completed,
                                                                  result.targets_total))

            # Do any post-processing
            if self._experiments[self._idx].process_messages():
                self.get_logger().info('messages processor returned True, STOPPING')
            else:
                # Clear goal_handle
                self._goal_handle = None

                # Step 4
                self.start_next_run_or_experiment_or_done()
        else:
            self.get_logger().warn('goal failed with status {0}, STOPPING'.format(status))

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
