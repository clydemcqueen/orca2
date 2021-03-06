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
Analyze and plot orca_msgs/msg/Control messages, looking only at the AUV motion segments.

Write a new plot when the motion segment changes.

Usage:

ros2 run orca_base plot_auv_segments.py
"""

import queue
import threading
import time
from typing import List

import matplotlib
from orca_msgs.msg import Control, MissionState, PolarObservation
from orca_util import get_yaw, norm_angle, seconds, set_ylim_with_min_range
import rclpy
from rclpy.node import Node
import rclpy.time

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

# Turn off flake8 checking for this late import
import matplotlib.pyplot as plt  # noqa: E402,I100


def plot_velo(ax, name, x_values, y_values):
    ax.set_title(name)
    ax.set_ylim(-0.55, 0.55)
    ax.set_xticklabels([])
    ax.plot(x_values, y_values)


def plot_pose(ax, name, plan_name, est_name, plan_x_values, est_x_values, plan_y_values,
              est_y_values):
    ax.plot(plan_x_values, plan_y_values, label=plan_name)
    ax.plot(est_x_values, est_y_values, label=est_name, marker='+', ls='')
    ax.set_xticklabels([])
    ax.set_title(name)
    ax.legend()
    set_ylim_with_min_range(ax, 0.2)


def plot_error(ax, name, x_values, y_values):
    ax.set_title(name)
    ax.set_ylim(-0.3, 0.3)
    ax.set_xticklabels([])
    ax.plot(x_values, y_values)


def plot_effort(ax, name, x_values, y_values):
    # Compute +/- transitions
    pos_neg = 0
    for c, n in zip(y_values, y_values[1:]):
        if c > 0. > n:
            pos_neg += 1
        elif c < 0. < n:
            pos_neg += 1

    ax.set_title('{}: pos_neg={}'.format(name, pos_neg))
    ax.set_ylim(-0.1, 0.1)
    ax.set_xticklabels([])
    ax.plot(x_values, y_values)


class Plotter(object):

    def __init__(self, filename):
        self._control_msgs: List[Control] = []
        self._filename = filename
        print('collecting messages for {}'.format(self._filename))

    def add_msg(self, msg):
        self._control_msgs.append(msg)

    def plot(self):
        if len(self._control_msgs) < 2:
            print('WARNING {} has too few messages! skipping'.format(self._filename))
            return

        print('plotting {}'.format(self._filename))
        start_time = time.process_time()

        # Create a figure and 16 subplots:
        # 4 for velocity
        # 4 for plan vs est
        # 4 for error in
        # 4 for efforts
        fig, ((axvx, axvy, axvz, axvw), (axpx, axpy, axpz, axpw),
              (axex, axey, axez, axew), (axff, axfs, axfv, axfw)) = plt.subplots(4, 4)

        # x axis for all plots == time
        # For most plots all all messages will be plotted
        all_stamps = [seconds(msg.header.stamp) for msg in self._control_msgs]

        # Warn on a large gap in timestamps -- this shouldn't happen with threading!
        gaps = [second - first for first, second in zip(all_stamps[:-1], all_stamps[1:])]
        largest_gap = max(gaps)
        if largest_gap > 0.1:
            print('WARNING large time gap {:.2f}s'.format(largest_gap))

        # For pose-based segments plot x, y, z, yaw
        # For observation-based segments plot distance, z and bearing
        if self._control_msgs[0].mission.segment_type == MissionState.OBS_RTM or \
                self._control_msgs[0].mission.segment_type == MissionState.OBS_MTM:

            # We expect 1 planned observation -- the marker we're tracking
            # The number of observations in estimate may vary from 0 to the total number of markers
            plan_obs_sizes = [len(msg.mission.pose.fp.observations.observations) for msg in
                              self._control_msgs]
            est_obs_sizes = [len(msg.estimate.observations.observations) for msg in
                             self._control_msgs]
            fewest_plan = min(plan_obs_sizes)
            most_plan = max(plan_obs_sizes)
            fewest_est = min(est_obs_sizes)
            most_est = max(est_obs_sizes)

            if fewest_plan != 1 or most_plan != 1:
                print('ERROR expected 1 planned observation, found min {}, max {}'.format(
                    fewest_plan, most_plan))
                return

            print('len(estimate_observations) varies from {} to {}'.format(fewest_est, most_est))

            # The marker we're tracking
            marker_id = self._control_msgs[0].mission.pose.fp.observations.observations[0].id

            # We may not find the marker in msg.estimate.observations,
            # so the # of data points may be smaller
            est_obs_stamps = []
            est_obs_distance_values = []
            est_obs_bearing_values = []
            error_distance_values = []
            error_bearing_values = []
            for msg in self._control_msgs:
                # Get distance and bearing to the planned observation
                plan_distance = msg.mission.pose.fp.observations.polar_observations[0].distance
                plan_bearing = msg.mission.pose.fp.observations.polar_observations[0].bearing

                # Try to find the marker in the estimate
                polar_obs: PolarObservation  # Type hint
                for polar_obs in msg.estimate.observations.polar_observations:
                    if polar_obs.id == marker_id:
                        est_obs_stamps.append(seconds(msg.header.stamp))
                        est_obs_distance_values.append(polar_obs.distance)
                        est_obs_bearing_values.append(polar_obs.bearing)
                        error_distance_values.append(polar_obs.distance - plan_distance)
                        error_bearing_values.append(polar_obs.bearing - plan_bearing)

            # Plot velocity data
            # No planned velocity for observations
            plot_velo(axvz, 'velo z', all_stamps,
                      [msg.mission.twist.linear.z for msg in self._control_msgs])

            # Plot pose data
            plot_pose(axpx, 'obs distance', 'plan distance', 'est distance', all_stamps,
                      est_obs_stamps,
                      [msg.mission.pose.fp.observations.polar_observations[0].distance for msg in
                       self._control_msgs],
                      est_obs_distance_values)
            plot_pose(axpz, 'pose z', 'plan z', 'est z', all_stamps, all_stamps,
                      [msg.mission.pose.position.z for msg in self._control_msgs],
                      [msg.estimate.pose.position.z for msg in self._control_msgs])
            plot_pose(axpw, 'obs bearing', 'plan bearing', 'est bearing', all_stamps,
                      est_obs_stamps,
                      [msg.mission.pose.fp.observations.polar_observations[0].bearing for msg in
                       self._control_msgs],
                      est_obs_bearing_values)

            # Plot error data
            plot_error(axex, 'error distance', est_obs_stamps, error_distance_values)
            plot_error(axez, 'error z', all_stamps,
                       [msg.estimate.pose.position.z - msg.mission.pose.position.z for msg in
                        self._control_msgs])
            plot_error(axew, 'error bearing', est_obs_stamps, error_bearing_values)

        else:

            # Plot velocity data
            plot_velo(axvx, 'velo x', all_stamps,
                      [msg.mission.twist.linear.x for msg in self._control_msgs])
            plot_velo(axvy, 'velo y', all_stamps,
                      [msg.mission.twist.linear.y for msg in self._control_msgs])
            plot_velo(axvz, 'velo z', all_stamps,
                      [msg.mission.twist.linear.z for msg in self._control_msgs])
            plot_velo(axvw, 'velo yaw', all_stamps,
                      [msg.mission.twist.angular.z for msg in self._control_msgs])

            # Plot pose data
            plot_pose(axpx, 'pose x', 'plan x', 'est x', all_stamps, all_stamps,
                      [msg.mission.pose.fp.pose.pose.position.x for msg in self._control_msgs],
                      [msg.estimate.pose.pose.position.x for msg in self._control_msgs])
            plot_pose(axpy, 'pose y', 'plan y', 'est y', all_stamps, all_stamps,
                      [msg.mission.pose.fp.pose.pose.position.y for msg in self._control_msgs],
                      [msg.estimate.pose.pose.position.y for msg in self._control_msgs])
            plot_pose(axpz, 'pose z', 'plan z', 'est z', all_stamps, all_stamps,
                      [msg.mission.pose.fp.pose.pose.position.z for msg in self._control_msgs],
                      [msg.estimate.pose.pose.position.z for msg in self._control_msgs])
            plot_pose(axpw, 'pose yaw', 'plan yaw', 'est yaw', all_stamps, all_stamps,
                      [get_yaw(msg.mission.pose.fp.pose.pose.orientation) for msg in
                       self._control_msgs],
                      [get_yaw(msg.estimate.pose.pose.orientation) for msg in self._control_msgs])

            # Plot error data
            plot_error(axex, 'error x', all_stamps,
                       [
                           msg.estimate.pose.pose.position.x -
                           msg.mission.pose.fp.pose.pose.position.x
                           for msg in
                           self._control_msgs])
            plot_error(axey, 'error y', all_stamps,
                       [
                           msg.estimate.pose.pose.position.x -
                           msg.mission.pose.fp.pose.pose.position.y
                           for msg in
                           self._control_msgs])
            plot_error(axez, 'error z', all_stamps,
                       [
                           msg.estimate.pose.pose.position.x -
                           msg.mission.pose.fp.pose.pose.position.z
                           for msg in
                           self._control_msgs])
            plot_error(axew, 'error yaw', all_stamps, [norm_angle(
                get_yaw(msg.estimate.pose.pose.orientation) - get_yaw(
                    msg.mission.pose.fp.pose.pose.orientation)) for
                msg in self._control_msgs])

        # Plot effort data
        plot_effort(axff, 'forward', all_stamps,
                    [msg.efforts.forward for msg in self._control_msgs])
        plot_effort(axfs, 'strafe', all_stamps, [msg.efforts.strafe for msg in self._control_msgs])
        plot_effort(axfv, 'vertical', all_stamps,
                    [msg.efforts.vertical for msg in self._control_msgs])
        plot_effort(axfw, 'yaw', all_stamps, [msg.efforts.yaw for msg in self._control_msgs])

        # Set figure title
        fig.suptitle('{} messages, {}'.format(len(self._control_msgs), self._filename))

        # Write the PDF file twice:
        # -- once with a unique name for later analysis
        # -- again with a repeated name so I can watch the plots live
        plt.savefig(self._filename)
        plt.savefig('plot_auv_segments.pdf')

        # Close the figure to reclaim the memory
        plt.close(fig)

        stop_time = time.process_time()
        print('finished {}, elapsed time {:.2f}s'.format(self._filename, stop_time - start_time))


def consumer(q: queue.Queue):
    while True:
        plotter: Plotter = q.get()
        plotter.plot()


class PlannerStatus(object):

    def __init__(self, msg: Control):
        self.global_plan_idx = msg.global_plan_idx

        self.targets_total = msg.mission.targets_total
        self.target_idx = msg.mission.target_idx
        self.target_marker_id = msg.mission.target_marker_id

        self.planner = msg.mission.planner
        self.local_plan_idx = msg.mission.local_plan_idx

        self.segments_total = msg.mission.segments_total
        self.segment_idx = msg.mission.segment_idx
        self.segment_info = msg.mission.segment_info
        self.segment_type = msg.mission.segment_type

    def different_segment(self, other):
        return self.global_plan_idx != other.global_plan_idx or \
               self.target_idx != other.target_idx or \
               self.local_plan_idx != other.local_plan_idx or \
               self.segment_idx != other.segment_idx

    def segment_name(self):
        if self.segment_type == MissionState.PAUSE:
            return 'pause'
        if self.segment_type == MissionState.POSE_VERTICAL:
            return 'pose_vertical'
        if self.segment_type == MissionState.POSE_ROTATE:
            return 'pose_rotate'
        if self.segment_type == MissionState.POSE_LINE:
            return 'pose_line'
        if self.segment_type == MissionState.POSE_COMBO:
            return 'pose_combo'
        if self.segment_type == MissionState.OBS_RTM:
            return 'obs_rtm'
        if self.segment_type == MissionState.OBS_MTM:
            return 'obs_mtm'
        return 'none'

    def filename(self):
        return 'g{}_t{}_l{}_s{}_{}.pdf'.format(self.global_plan_idx, self.target_idx,
                                               self.local_plan_idx, self.segment_idx,
                                               self.segment_name())


class PlotControlNode(Node):

    def __init__(self):
        super().__init__('plot_control')
        self._control_sub = self.create_subscription(Control, '/control',
                                                     self.control_callback, 10)

        self._plot = None
        self._prev_status = None

        # Use the producer-consumer model
        # We are the producer -- we collect messages in self._plot, then add it to the queue
        self._q = queue.Queue()

        # The consumer takes Plotter objects off the queue and creates the plot
        # This takes ~2s, which will cause the node to drop messages
        # Run the consumer in it's own thread to avoid this
        self._thread = threading.Thread(target=consumer, args=(self._q,))
        self._thread.start()

    def control_callback(self, msg: Control):
        # Ignore ROV messages
        if msg.mode != Control.AUV:
            return

        curr_status = PlannerStatus(msg)

        # Bootstrap
        if self._plot is None:
            self._plot = Plotter(curr_status.filename())
            self._prev_status = curr_status

        # Plot when segment changes
        if curr_status.different_segment(self._prev_status):
            self._q.put(self._plot)
            self._plot = Plotter(curr_status.filename())
            self._prev_status = curr_status

        # Add message to the plotter
        self._plot.add_msg(msg)


def main():
    print('backend is', plt.get_backend())

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [24., 15.]

    rclpy.init()
    node = PlotControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
        # TODO notify consumer thread for a clean shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
