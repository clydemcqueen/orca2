#!/usr/bin/env python3

"""
Analyze and plot orca_msgs/msg/Control messages, looking only at the AUV motion segments
Write a new plot when the motion segment changes

Usage:

ros2 run orca_base plot_auv_segments.py
"""

import matplotlib

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
import math
from orca_msgs.msg import Control
import rclpy
import rclpy.time
from rclpy.node import Node
import transformations as xf
from typing import List


def normalize_angle(x):
    x = x % (2 * math.pi)  # force in range [0, 2 pi)
    if x > math.pi:  # move to [-pi, pi)
        x -= 2 * math.pi
    return x


def get_yaw(q: Quaternion) -> float:
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy[2]


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
        self._control_msgs: List[Control] = []
        self._control_sub = self.create_subscription(Control, '/control', self.control_callback, 10)
        self._prev_status = None

    def control_callback(self, msg: Control):
        # Ignore ROV messages
        if msg.mode != Control.AUV:
            return

        curr_status = PlannerStatus(msg)

        # Bootstrap
        if self._prev_status is None:
            self._prev_status = curr_status

        # Plot when segment changes
        if curr_status.different_segment(self._prev_status):
            if len(self._control_msgs) < 2:
                print('error -- too few messages! skipping')
            else:
                filename = self._prev_status.filename()
                print(filename)

                # Plot and save to a unique file
                # self.plot_msgs('{} messages, {}'.format(len(self._control_msgs), filename), filename)

                # Plot and overwrite a single file
                self.plot_msgs('{} messages, {}'.format(len(self._control_msgs), filename), 'plot_control.pdf')

            # Clear messages
            self._control_msgs.clear()

            # Set prev
            self._prev_status = curr_status

        # Add message to the queue
        self._control_msgs.append(msg)

    def plot_msgs(self, title, filename):
        # Create a figure and 12 subplots:
        # 4 for velocity in the world frame
        # 4 for error in the world frame
        # 4 for efforts in the body frame
        fig, ((axvx, axvy, axvz, axvw), (axex, axey, axez, axew), (axff, axfs, axfv, axfw)) = plt.subplots(3, 4)

        # Plot velocity
        velo_axes = [axvx, axvy, axvz, axvw]
        velo_names = ['velo x', 'velo y', 'velo z', 'velo yaw']
        velo_values = [[msg.plan_twist.linear.x for msg in self._control_msgs],
                       [msg.plan_twist.linear.y for msg in self._control_msgs],
                       [msg.plan_twist.linear.z for msg in self._control_msgs],
                       [msg.plan_twist.angular.z for msg in self._control_msgs]]
        for ax, name, values in zip(velo_axes, velo_names, velo_values):
            ax.set_title(name)
            ax.set_ylim(-0.55, 0.55)
            ax.set_xticklabels([])
            ax.plot(values)

        # Plot error
        error_axes = [axex, axey, axez, axew]
        error_names = ['error x', 'error y', 'error z', 'error yaw']
        error_values = [[msg.estimate_pose.position.x - msg.plan_pose.position.x for msg in self._control_msgs],
                        [msg.estimate_pose.position.y - msg.plan_pose.position.y for msg in self._control_msgs],
                        [msg.estimate_pose.position.z - msg.plan_pose.position.z for msg in self._control_msgs],
                        [normalize_angle(get_yaw(msg.estimate_pose.orientation) - get_yaw(msg.plan_pose.orientation))
                         for msg in self._control_msgs]]
        for ax, name, values in zip(error_axes, error_names, error_values):
            ax.set_title(name)
            ax.set_ylim(-0.3, 0.3)
            ax.set_xticklabels([])
            ax.plot(values)

        # Plot efforts
        effort_axes = [axff, axfs, axfv, axfw]
        effort_names = ['forward', 'strafe', 'vertical', 'yaw']
        effort_values = [[msg.efforts.forward for msg in self._control_msgs],
                         [msg.efforts.strafe for msg in self._control_msgs],
                         [msg.efforts.vertical for msg in self._control_msgs],
                         [msg.efforts.yaw for msg in self._control_msgs]]
        for ax, name, values in zip(effort_axes, effort_names, effort_values):
            # Compute +/- transitions
            pos_neg = 0
            for c, n in zip(values, values[1:]):
                if c > 0. > n:
                    pos_neg += 1
                elif c < 0. < n:
                    pos_neg += 1

            ax.set_title('{}: pos_neg={}'.format(name, pos_neg))
            ax.set_ylim(-0.1, 0.1)
            ax.set_xticklabels([])
            ax.plot(values)

        # Set figure title
        fig.suptitle(title)

        # [Over]write PDF to disk
        plt.savefig(filename)

        # Close the figure to reclaim the memory
        plt.close(fig)


def main():
    print('backend is', plt.get_backend())

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [24., 12.]

    rclpy.init()
    node = PlotControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
