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

r"""
Analyze and plot the output of the filter.

TODO(clyde): plotting time is 0.7s, so we're dropping messages, add threading

Usage:

ros2 run orca_base plot_filter_fp.py 8 \
--ros-args -r /pre_filter:=/forward_camera/fp -r /post_filter:=/filtered_fp
"""

import math
import statistics
import sys
import time
from typing import List

import matplotlib
from nav_msgs.msg import Odometry
import nees_fp
import numpy as np
from orca_msgs.msg import Control, Depth, FiducialPoseStamped
from orca_util import q_to_rpy, seconds, set_ylim_with_min_range
import rclpy
from rclpy.node import Node

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

# Turn off flake8 checking for this late import
import matplotlib.pyplot as plt  # noqa: E402,I100


def diag_index(dim):
    return dim * 7


# Test for valid covariance, in this case the diagonal needs to be >=0
def valid_covariance(msg: FiducialPoseStamped):
    for index in range(0, 5):
        if msg.fp.pose.covariance[diag_index(index)] < 0:
            return False
    return True


def plot_subplot(subplot, name,
                 depth_xs, depth_values, depth_sds,
                 pre_xs, pre_values, pre_sds,
                 post_xs, post_values, post_sds,
                 gt_xs, gt_values,
                 plan_xs, plan_values):
    """Plot data in a single subplot."""
    plot_error = False  # Plot points with error bars, best with queue_for <= 2s

    if gt_xs and gt_values:
        subplot.plot(gt_xs, gt_values, label='truth')

    if plot_error:
        subplot.errorbar(post_xs, post_values, yerr=post_sds, marker='+', ls='', alpha=1.0,
                         elinewidth=1, label='filter')
    else:
        subplot.plot(post_xs, post_values, label='filter')

    if pre_xs and pre_values and pre_sds:
        if plot_error:
            subplot.errorbar(pre_xs, pre_values, yerr=pre_sds, marker='x', ls='', alpha=0.8,
                             elinewidth=1, label='pre')
        else:
            subplot.plot(pre_xs, pre_values, label='pre')

    if depth_xs and depth_values and depth_sds:
        if plot_error:
            subplot.errorbar(depth_xs, depth_values, yerr=depth_sds, marker='o', ls='', alpha=0.8,
                             elinewidth=1, label='depth')
        else:
            subplot.plot(depth_xs, depth_values, label='depth')

    if plan_xs and plan_values:
        subplot.plot(plan_xs, plan_values, label='plan')

    set_ylim_with_min_range(subplot, 0.05)
    # subplot.set_xticklabels([])
    subplot.legend()

    if post_values and len(post_values) > 1:
        if pre_values and len(pre_values) > 1:
            pre_u = statistics.mean(pre_values)
            pre_s = statistics.stdev(pre_values, pre_u)
            post_u = statistics.mean(post_values)
            post_s = statistics.stdev(post_values, post_u)
            subplot.set_title(
                '{}, pre ({:.3f}, {:.3f}), post ({:.3f}, {:.3f})'.format(name, pre_u, pre_s,
                                                                         post_u, post_s))
        else:
            post_u = statistics.mean(post_values)
            post_s = statistics.stdev(post_values, post_u)
            subplot.set_title('{}, post ({:.3f}, {:.3f})'.format(name, post_u, post_s))
    else:
        subplot.set_title('{}, no stats'.format(name))


class PlotFilterNode(Node):

    def __init__(self, queue_for: float):
        super().__init__('plot_filter')
        self._queue_for = queue_for
        self._first_time = None
        self._last_time = None
        self._depth_msgs: List[Depth] = []
        self._pre_msgs: List[FiducialPoseStamped] = []
        self._post_msgs: List[FiducialPoseStamped] = []
        self._gt_msgs: List[Odometry] = []
        self._control_msgs: List[Control] = []

        plot_depth = True
        plot_gt = False
        plot_plan = True

        if plot_depth:
            self._depth_sub = self.create_subscription(Depth, '/depth', self.depth_callback, 10)
        self._fcam_sub = self.create_subscription(FiducialPoseStamped, '/pre_filter',
                                                  self.pre_callback, 10)
        self._post_sub = self.create_subscription(FiducialPoseStamped, '/post_filter',
                                                  self.post_callback, 10)
        if plot_gt:
            self._gt_sub = self.create_subscription(Odometry, '/ground_truth',
                                                    self.gt_callback, 10)
        if plot_plan:
            self._control_sub = self.create_subscription(Control, '/control',
                                                         self.control_callback, 10)

    def reset(self):
        self._first_time = None
        self._last_time = None
        self._depth_msgs = []
        self._pre_msgs = []
        self._post_msgs = []
        self._gt_msgs = []
        self._control_msgs = []

    def depth_callback(self, msg: Depth):
        self._depth_msgs.append(msg)
        self.process(msg)

    def pre_callback(self, msg: FiducialPoseStamped):
        if valid_covariance(msg):
            self._pre_msgs.append(msg)
            self.process(msg)
        else:
            print('INVALID PRE COVARIANCE, DROP')

    def post_callback(self, msg: FiducialPoseStamped):
        if valid_covariance(msg):
            self._post_msgs.append(msg)
            self.process(msg)
        else:
            print('INVALID POST COVARIANCE, DROP')

    def gt_callback(self, msg: Odometry):
        self._gt_msgs.append(msg)
        self.process(msg)

    def control_callback(self, msg: Control):
        self._control_msgs.append(msg)
        self.process(msg)

    def process(self, msg):
        s = seconds(msg.header.stamp)

        # Update first, last
        if self._first_time is None or s < self._first_time:
            self._first_time = s
        if self._last_time is None or s > self._last_time:
            self._last_time = s

        # Plot messages?
        if self._last_time - self._first_time > self._queue_for:
            self.plot_msgs()
            self.reset()

    def calc_nees(self) -> float:
        """Calc mean NEES value."""
        nees_values = nees_fp.nees(self._post_msgs, self._gt_msgs)
        if nees_values:
            return float(np.mean(nees_values))
        else:
            return -1.

    def plot_msgs(self):
        """Plot queued messages."""
        print('plotting plot_filter.pdf')
        start_time = time.process_time()

        # Convert quaternions to Euler angles
        pre_pose_rpys = [q_to_rpy(msg.fp.pose.pose.orientation) for msg in self._pre_msgs]
        post_pose_rpys = [q_to_rpy(msg.fp.pose.pose.orientation) for msg in self._post_msgs]
        gt_pose_rpys = [q_to_rpy(msg.pose.pose.orientation) for msg in self._gt_msgs]
        plan_pose_rpys = [q_to_rpy(msg.mission.pose.fp.pose.pose.orientation)
                          for msg in self._control_msgs]

        # Create a figure and 6 subplots
        fig, ((axpx, axpy, axpz), (axproll, axppitch, axpyaw)) = plt.subplots(2, 3)

        # X values are seconds from the first message
        depth_xs = [seconds(msg.header.stamp) - self._first_time for msg in self._depth_msgs]
        pre_xs = [seconds(msg.header.stamp) - self._first_time for msg in self._pre_msgs]
        post_xs = [seconds(msg.header.stamp) - self._first_time for msg in self._post_msgs]
        gt_xs = [seconds(msg.header.stamp) - self._first_time for msg in self._gt_msgs]
        plan_xs = [seconds(msg.header.stamp) - self._first_time for msg in self._control_msgs]

        subplots = [axpx, axpy, axpz, axproll, axppitch, axpyaw]
        names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

        # Build 6 lists of y values, 1 for each subplot
        depth_valuess = [None, None, [msg.z for msg in self._depth_msgs],
                         None, None, None]

        depth_sdss = [None, None, [math.sqrt(msg.z_variance) for msg in self._depth_msgs],
                      None, None, None]

        pre_valuess = [[msg.fp.pose.pose.position.x for msg in self._pre_msgs],
                       [msg.fp.pose.pose.position.y for msg in self._pre_msgs],
                       [msg.fp.pose.pose.position.z for msg in self._pre_msgs],
                       [rpy[0] for rpy in pre_pose_rpys],
                       [rpy[1] for rpy in pre_pose_rpys],
                       [rpy[2] for rpy in pre_pose_rpys]]

        pre_sdss = [[math.sqrt(msg.fp.pose.covariance[diag_index(0)]) for msg in self._pre_msgs],
                    [math.sqrt(msg.fp.pose.covariance[diag_index(1)]) for msg in self._pre_msgs],
                    [math.sqrt(msg.fp.pose.covariance[diag_index(2)]) for msg in self._pre_msgs],
                    [math.sqrt(msg.fp.pose.covariance[diag_index(3)]) for msg in self._pre_msgs],
                    [math.sqrt(msg.fp.pose.covariance[diag_index(4)]) for msg in self._pre_msgs],
                    [math.sqrt(msg.fp.pose.covariance[diag_index(5)]) for msg in self._pre_msgs]]

        post_valuess = [[msg.fp.pose.pose.position.x for msg in self._post_msgs],
                        [msg.fp.pose.pose.position.y for msg in self._post_msgs],
                        [msg.fp.pose.pose.position.z for msg in self._post_msgs],
                        [rpy[0] for rpy in post_pose_rpys],
                        [rpy[1] for rpy in post_pose_rpys],
                        [rpy[2] for rpy in post_pose_rpys]]

        post_sdss = [[math.sqrt(msg.fp.pose.covariance[diag_index(0)]) for msg in self._post_msgs],
                     [math.sqrt(msg.fp.pose.covariance[diag_index(1)]) for msg in self._post_msgs],
                     [math.sqrt(msg.fp.pose.covariance[diag_index(2)]) for msg in self._post_msgs],
                     [math.sqrt(msg.fp.pose.covariance[diag_index(3)]) for msg in self._post_msgs],
                     [math.sqrt(msg.fp.pose.covariance[diag_index(4)]) for msg in self._post_msgs],
                     [math.sqrt(msg.fp.pose.covariance[diag_index(5)]) for msg in self._post_msgs]]

        gt_valuess = [[msg.pose.pose.position.x for msg in self._gt_msgs],
                      [msg.pose.pose.position.y for msg in self._gt_msgs],
                      [msg.pose.pose.position.z for msg in self._gt_msgs],
                      [rpy[0] for rpy in gt_pose_rpys],
                      [rpy[1] for rpy in gt_pose_rpys],
                      [rpy[2] for rpy in gt_pose_rpys]]

        plan_valuess = [[msg.mission.pose.fp.pose.pose.position.x for msg in self._control_msgs],
                        [msg.mission.pose.fp.pose.pose.position.y for msg in self._control_msgs],
                        [msg.mission.pose.fp.pose.pose.position.z for msg in self._control_msgs],
                        None,
                        None,
                        [rpy[2] for rpy in plan_pose_rpys]]

        # Plot everything
        for subplot, name, depth_values, depth_sds, pre_values, pre_sds, post_values, post_sds, \
            gt_values, plan_values in zip(subplots, names, depth_valuess, depth_sdss, pre_valuess,
                                          pre_sdss, post_valuess, post_sdss, gt_valuess,
                                          plan_valuess):
            plot_subplot(subplot, name,
                         depth_xs, depth_values, depth_sds,
                         pre_xs, pre_values, pre_sds,
                         post_xs, post_values, post_sds,
                         gt_xs, gt_values, plan_xs, plan_values)

        # Calc average NEES
        ave_nees = self.calc_nees()
        if ave_nees < 0:
            nees_str = 'no estimates'
        elif ave_nees < 12:
            nees_str = 'ave NEES {:.3f}, SUCCESS'.format(ave_nees)
        else:
            nees_str = 'ave NEES {:.3f}, FAILURE'.format(ave_nees)

        # Set figure title
        fig.suptitle(
            'UKF status, {} second(s), with (mean, stddev), {}'.format(self._queue_for, nees_str))

        # [Over]write PDF to disk
        plt.savefig('plot_filter.pdf')

        # Close the figure to reclaim the memory
        plt.close(fig)

        # If elapsed time > 0.3s we're may be dropping messages
        # Add a plotting thread to fix
        stop_time = time.process_time()
        print('elapsed time {:.2f}s'.format(stop_time - start_time))


def main(args=None):
    print('backend is', plt.get_backend())

    # Set figure size (width and height in inches)
    plt.rcParams['figure.figsize'] = [21., 10.]

    queue_for = 8
    queue_for = int(sys.argv[1])
    print('queue for {} seconds'.format(queue_for))

    rclpy.init()
    node = PlotFilterNode(queue_for)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
