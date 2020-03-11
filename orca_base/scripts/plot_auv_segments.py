#!/usr/bin/env python3

"""
Analyze and plot orca_msgs/msg/Control messages, looking only at the AUV motion segments
Write a new plot when the motion segment changes

Usage:

ros2 run orca_base plot_auv_segments.py
"""

import queue
import threading
import time

import matplotlib

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

import matplotlib.pyplot as plt
from orca_msgs.msg import Control
import rclpy
import rclpy.time
from rclpy.node import Node
from typing import List
from orca_util import normalize_angle, get_yaw, seconds

MIN_RANGE = 0.2


# Set ylim to some reasonable values
def set_ylim_with_min_range(ax):
    limits = ax.get_ylim()
    range = limits[1] - limits[0]

    if range < MIN_RANGE:
        adj = (MIN_RANGE - range) / 2.0
        ax.set_ylim(limits[0] - adj, limits[1] + adj)


class Plotter(object):

    def __init__(self, filename):
        self._control_msgs: List[Control] = []
        self._filename = filename
        print('collecting messages for {}'.format(self._filename))

    def add_msg(self, msg):
        self._control_msgs.append(msg)

    def plot(self):
        if len(self._control_msgs) < 2:
            print('error -- {} has too few messages! skipping'.format(self._filename))
            return

        print('plotting {}'.format(self._filename))
        start_time = time.process_time()

        # Create a figure and 12 subplots:
        # 4 for velocity in the world frame
        # 4 for plan vs est in the world frame
        # 4 for error in the world frame
        # 4 for efforts in the body frame
        fig, ((axvx, axvy, axvz, axvw), (axpx, axpy, axpz, axpw),
              (axex, axey, axez, axew), (axff, axfs, axfv, axfw)) = plt.subplots(4, 4)

        # x axis for all plots == time
        x_values = [seconds(msg.header.stamp) for msg in self._control_msgs]

        # Warn on a large gap -- this shouldn't happen with threading
        gaps = [second - first for first, second in zip(x_values[:-1], x_values[1:])]
        largest_gap = max(gaps)
        if largest_gap > 0.1:
            print('WARNING large time stamp gap {:.2f}s'.format(largest_gap))

        plan_names = ['plan x', 'plan y', 'plan z', 'plan yaw']
        est_names = ['est x', 'est y', 'est z', 'est yaw']

        # Plot velocity
        # TODO plot estimated velo as well
        velo_axes = [axvx, axvy, axvz, axvw]
        velo_names = ['velo x', 'velo y', 'velo z', 'velo yaw']
        plan_velo_valuess = [[msg.plan_twist.linear.x for msg in self._control_msgs],
                             [msg.plan_twist.linear.y for msg in self._control_msgs],
                             [msg.plan_twist.linear.z for msg in self._control_msgs],
                             [msg.plan_twist.angular.z for msg in self._control_msgs]]
        for ax, name, plan_velo_values in zip(velo_axes, velo_names, plan_velo_valuess):
            ax.set_title(name)
            ax.set_ylim(-0.55, 0.55)
            ax.set_xticklabels([])
            ax.plot(x_values, plan_velo_values)

        # Plot plan vs estimate in the same graph
        pose_axes = [axpx, axpy, axpz, axpw]
        pose_names = ['pose x', 'pose y', 'pose z', 'pose yaw']
        plan_pose_valuess = [[msg.plan_pose.position.x for msg in self._control_msgs],
                             [msg.plan_pose.position.y for msg in self._control_msgs],
                             [msg.plan_pose.position.z for msg in self._control_msgs],
                             [get_yaw(msg.plan_pose.orientation) for msg in self._control_msgs]]
        est_pose_valuess = [[msg.estimate_pose.position.x for msg in self._control_msgs],
                            [msg.estimate_pose.position.y for msg in self._control_msgs],
                            [msg.estimate_pose.position.z for msg in self._control_msgs],
                            [get_yaw(msg.estimate_pose.orientation) for msg in self._control_msgs]]
        for ax, pose_name, plan_name, est_name, plan_pose_values, est_pose_values in zip(pose_axes, pose_names,
                                                                                         plan_names, est_names,
                                                                                         plan_pose_valuess,
                                                                                         est_pose_valuess):
            ax.plot(x_values, plan_pose_values, label=plan_name)
            ax.plot(x_values, est_pose_values, label=est_name)
            ax.set_xticklabels([])
            ax.set_title(pose_name)
            ax.legend()
            set_ylim_with_min_range(ax)

        # Plot error
        error_axes = [axex, axey, axez, axew]
        error_names = ['error x', 'error y', 'error z', 'error yaw']
        error_valuess = [[msg.estimate_pose.position.x - msg.plan_pose.position.x for msg in self._control_msgs],
                         [msg.estimate_pose.position.y - msg.plan_pose.position.y for msg in self._control_msgs],
                         [msg.estimate_pose.position.z - msg.plan_pose.position.z for msg in self._control_msgs],
                         [normalize_angle(get_yaw(msg.estimate_pose.orientation) - get_yaw(msg.plan_pose.orientation))
                          for msg in self._control_msgs]]
        for ax, name, error_values in zip(error_axes, error_names, error_valuess):
            ax.set_title(name)
            ax.set_ylim(-0.3, 0.3)
            ax.set_xticklabels([])
            ax.plot(x_values, error_values)

        # Plot efforts
        effort_axes = [axff, axfs, axfv, axfw]
        effort_names = ['forward', 'strafe', 'vertical', 'yaw']
        effort_valuess = [[msg.efforts.forward for msg in self._control_msgs],
                          [msg.efforts.strafe for msg in self._control_msgs],
                          [msg.efforts.vertical for msg in self._control_msgs],
                          [msg.efforts.yaw for msg in self._control_msgs]]
        for ax, name, effort_values in zip(effort_axes, effort_names, effort_valuess):
            # Compute +/- transitions
            pos_neg = 0
            for c, n in zip(effort_values, effort_values[1:]):
                if c > 0. > n:
                    pos_neg += 1
                elif c < 0. < n:
                    pos_neg += 1

            ax.set_title('{}: pos_neg={}'.format(name, pos_neg))
            ax.set_ylim(-0.1, 0.1)
            ax.set_xticklabels([])
            ax.plot(x_values, effort_values)

        # Set figure title
        fig.suptitle('{} messages, {}'.format(len(self._control_msgs), self._filename))

        # Write PDF file twice:
        # -- once with a unique name for detailed analysis
        # -- again with a repeated name so I can watch the plots live
        plt.savefig(self._filename)
        plt.savefig('plot_control.pdf')

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

        self.targets_total = msg.targets_total
        self.target_idx = msg.target_idx
        self.target_marker_id = msg.target_marker_id

        self.planner = msg.planner
        self.local_plan_idx = msg.local_plan_idx

        self.segments_total = msg.segments_total
        self.segment_idx = msg.segment_idx
        self.segment_info = msg.segment_info
        self.segment_type = msg.segment_type

    def different_segment(self, other):
        return self.global_plan_idx != other.global_plan_idx or \
               self.target_idx != other.target_idx or \
               self.local_plan_idx != other.local_plan_idx or \
               self.segment_idx != other.segment_idx

    def segment_name(self):
        if self.segment_type == Control.PAUSE:
            return 'pause'
        if self.segment_type == Control.POSE_VERTICAL:
            return 'pose_vertical'
        if self.segment_type == Control.POSE_ROTATE:
            return 'pose_rotate'
        if self.segment_type == Control.POSE_LINE:
            return 'pose_line'
        if self.segment_type == Control.POSE_COMBO:
            return 'pose_combo'
        if self.segment_type == Control.OBS_RTM:
            return 'obs_rtm'
        if self.segment_type == Control.OBS_MTM:
            return 'obs_mtm'
        return 'none'

    def filename(self):
        return 'g{}_t{}_l{}_s{}_{}.pdf'.format(self.global_plan_idx, self.target_idx, self.local_plan_idx,
                                               self.segment_idx, self.segment_name())


class PlotControlNode(Node):

    def __init__(self):
        super().__init__('plot_control')
        self._control_sub = self.create_subscription(Control, '/control', self.control_callback, 10)

        self._plot = None
        self._prev_status = None

        # Use the producer-consumer model
        # We are the producer -- we collect messages in self._plot
        # When the plotter has "enough" message, it gets added to the queue
        self._q = queue.Queue()

        # The consumer creates the plot
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
