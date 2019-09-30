#!/usr/bin/env python3

"""
Analyze and plot the output of a Kalman filter by subscribing to 2 Odometry messages: pre- and post-filter

Usage: ros2 run orca_base plot_filter.py /pre_filter:=/left_camera/odom /post_filter:=/filtered_odom
"""

import statistics
from typing import List

import matplotlib
import matplotlib.pyplot as plt
import rclpy
import transformations as xf
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from orca_msgs.msg import Barometer
from rclpy.node import Node

QUEUE_FOR = 1.0  # Seconds


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


def diag_index(dim):
    return dim * 7


def plot_subplot(subplot, name,
                 baro_xs, baro_values, baro_vars,
                 pre_xs, pre_values, pre_vars,
                 post_xs, post_values, post_vars):
    """Plot data in a single subplot"""

    if pre_xs and pre_values and pre_vars:
        subplot.errorbar(pre_xs, pre_values, yerr=pre_vars, marker='x', ls='', alpha=0.8, elinewidth=1, label='pre')

    subplot.plot(post_xs, post_values, label='post')

    post_s_los = [value - var for value, var in zip(post_values, post_vars)]
    post_s_his = [value + var for value, var in zip(post_values, post_vars)]
    subplot.fill_between(post_xs, post_s_los, post_s_his, color='gray', alpha=0.2)

    if baro_xs and baro_values and baro_vars:
        subplot.errorbar(baro_xs, baro_values, yerr=baro_vars, marker='o', ls='', label='baro')

    subplot.set_xticklabels([])
    subplot.legend()

    if post_values and len(post_values) > 1:
        if pre_values and len(pre_values) > 1:
            pre_u = statistics.mean(pre_values)
            pre_s = statistics.stdev(pre_values, pre_u)
            post_u = statistics.mean(post_values)
            post_s = statistics.stdev(post_values, post_u)
            subplot.set_title('{}, pre ({:.3f}, {:.3f}), post ({:.3f}, {:.3f})'.format(name, pre_u, pre_s, post_u, post_s))
        else:
            post_u = statistics.mean(post_values)
            post_s = statistics.stdev(post_values, post_u)
            subplot.set_title('{}, post ({:.3f}, {:.3f})'.format(name, post_u, post_s))
    else:
        subplot.set_title('{}, no stats'.format(name))


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class PlotFilterNode(Node):

    def __init__(self):
        super().__init__('plot_odom')
        self._first_time = None
        self._last_time = None
        self._baro_msgs: List[Barometer] = []
        self._pre_msgs: List[Odometry] = []
        self._post_msgs: List[Odometry] = []

        self._baro_sub = self.create_subscription(Barometer, '/barometer_adj', self.baro_callback, 5)
        self._pre_sub = self.create_subscription(Odometry, '/odom', self.pre_callback, 5)
        self._post_sub = self.create_subscription(Odometry, '/filtered_odom', self.post_callback, 5)

    def baro_callback(self, msg: Barometer):
        self._baro_msgs.append(msg)
        self.process(msg)

    def pre_callback(self, msg: Odometry):
        self._pre_msgs.append(msg)
        self.process(msg)

    def post_callback(self, msg: Odometry):
        self._post_msgs.append(msg)
        self.process(msg)

    def process(self, msg):
        s = seconds(msg.header.stamp)

        # Update 1st, last
        if self._first_time is None or s < self._first_time:
            self._first_time = s
        if self._last_time is None or s > self._last_time:
            self._last_time = s

        # Plot messages?
        if self._last_time - self._first_time > QUEUE_FOR:
            self.plot_msgs()

            # Reset
            self._first_time = None
            self._last_time = None
            self._baro_msgs: List[Barometer] = []
            self._pre_msgs: List[Odometry] = []
            self._post_msgs: List[Odometry] = []

    def plot_msgs(self):
        """Plot queued messages"""

        # Convert quaternions to Euler angles
        pre_pose_rpys = [q_to_rpy(pre.pose.pose.orientation) for pre in self._pre_msgs]
        post_pose_rpys = [q_to_rpy(post.pose.pose.orientation) for post in self._post_msgs]

        # Create a figure and 12 subplots, 6 for the pose and 6 for the twist
        fig, ((axpx, axpy, axpz), (axtx, axty, axtz),
              (axproll, axppitch, axpyaw), (axtroll, axtpitch, axtyaw)) = plt.subplots(4, 3)

        # Build lists of items to plot
        baro_xs = [seconds(msg.header.stamp) for msg in self._baro_msgs]
        pre_xs = [seconds(msg.header.stamp) for msg in self._pre_msgs]
        post_xs = [seconds(msg.header.stamp) for msg in self._post_msgs]
        subplots = [axpx, axpy, axpz, axtx, axty, axtz, axproll, axppitch, axpyaw, axtroll, axtpitch, axtyaw]
        names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'v roll', 'v pitch', 'v yaw']

        baro_valuess = [None, None, [msg.z for msg in self._baro_msgs],
                        None, None, None,
                        None, None, None,
                        None, None, None]

        baro_varss = [None, None, [msg.z_variance for msg in self._baro_msgs],
                      None, None, None,
                      None, None, None,
                      None, None, None]

        pre_valuess = [[msg.pose.pose.position.x for msg in self._pre_msgs],
                       [msg.pose.pose.position.y for msg in self._pre_msgs],
                       [msg.pose.pose.position.z for msg in self._pre_msgs],
                       None, None, None,
                       [rpy[0] for rpy in pre_pose_rpys],
                       [rpy[1] for rpy in pre_pose_rpys],
                       [rpy[2] for rpy in pre_pose_rpys],
                       None, None, None]

        pre_varss = [[msg.pose.covariance[diag_index(0)] for msg in self._pre_msgs],
                     [msg.pose.covariance[diag_index(1)] for msg in self._pre_msgs],
                     [msg.pose.covariance[diag_index(2)] for msg in self._pre_msgs],
                     None, None, None,
                     [msg.pose.covariance[diag_index(3)] for msg in self._pre_msgs],
                     [msg.pose.covariance[diag_index(4)] for msg in self._pre_msgs],
                     [msg.pose.covariance[diag_index(5)] for msg in self._pre_msgs],
                     None, None, None]

        post_valuess = [[msg.pose.pose.position.x for msg in self._post_msgs],
                        [msg.pose.pose.position.y for msg in self._post_msgs],
                        [msg.pose.pose.position.z for msg in self._post_msgs],
                        [msg.twist.twist.linear.x for msg in self._post_msgs],
                        [msg.twist.twist.linear.y for msg in self._post_msgs],
                        [msg.twist.twist.linear.z for msg in self._post_msgs],
                        [rpy[0] for rpy in post_pose_rpys],
                        [rpy[1] for rpy in post_pose_rpys],
                        [rpy[2] for rpy in post_pose_rpys],
                        [msg.twist.twist.angular.x for msg in self._post_msgs],
                        [msg.twist.twist.angular.y for msg in self._post_msgs],
                        [msg.twist.twist.angular.z for msg in self._post_msgs]]

        post_varss = [[msg.pose.covariance[diag_index(0)] for msg in self._post_msgs],
                      [msg.pose.covariance[diag_index(1)] for msg in self._post_msgs],
                      [msg.pose.covariance[diag_index(2)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(0)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(1)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(2)] for msg in self._post_msgs],
                      [msg.pose.covariance[diag_index(3)] for msg in self._post_msgs],
                      [msg.pose.covariance[diag_index(4)] for msg in self._post_msgs],
                      [msg.pose.covariance[diag_index(5)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(3)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(4)] for msg in self._post_msgs],
                      [msg.twist.covariance[diag_index(5)] for msg in self._post_msgs]]

        # Plot everything
        for subplot, name, baro_values, baro_vars, pre_values, pre_vars, post_values, post_vars in \
                zip(subplots, names, baro_valuess, baro_varss, pre_valuess, pre_varss, post_valuess, post_varss):
            plot_subplot(subplot, name,
                         baro_xs, baro_values, baro_vars,
                         pre_xs, pre_values, pre_vars,
                         post_xs, post_values, post_vars)

        # Set figure title
        fig.suptitle('pre- and post-filter odometry messages, {} second(s), with (mean, stddev)'.format(QUEUE_FOR))

        # [Over]write PDF to disk
        plt.savefig('plot_filter.pdf')

        # Close the figure to reclaim the memory
        plt.close(fig)


def main(args=None):
    """
    Updating plots is tricky.

    Method 1 (failed):
    -- turn interactive mode on using plt.ion()
    -- call plt.pause(0.01) to allow/force an update

    Unfortunately, plt.pause() appears to call activateWindow() on the Qt5Agg backend. This brings the Qt5Agg window
    to the front and grabs focus, interrupting anything else you're doing. This is annoying.

    Method 2 (current):
    -- use the PDF backend to write plots to PDF files
    -- view the files using `evince fig.pdf`

    Evince watches the file and reloads whenever it changes. The downside is that the plot is a static image, so
    resizing the window doesn't do what you expect.

    Perhaps worth investigating:
    -- custom mypause function that doesn't call activateWindow()
    -- matplotlib animations
    """

    print('backend was', plt.get_backend())
    matplotlib.use('pdf')
    print('backend is', plt.get_backend())

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [21., 14.]

    rclpy.init(args=args)
    node = PlotFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()