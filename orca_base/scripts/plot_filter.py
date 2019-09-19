#!/usr/bin/env python3

"""
Analyze and plot the output of a Kalman filter by subscribing to 2 Odometry messages: pre- and post-filter

Usage: ros2 run orca_base plot_filter.py /pre_filter:=/left_camera/odom /post_filter:=/filtered_odom
"""

import statistics
from typing import List

import matplotlib
import matplotlib.pyplot as plt
import message_filters
import rclpy
import transformations as xf
from nav_msgs.msg import Odometry
from rclpy.node import Node

NUM_MESSAGES = 30


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


class PlotFilterNode(Node):

    def __init__(self):
        super().__init__('plot_odom')
        self._pre_msgs: List[Odometry] = []
        self._post_msgs: List[Odometry] = []

        # Listen to synchronized pre- and post-filter messages
        self._time_sync = message_filters.TimeSynchronizer([
            message_filters.Subscriber(self, Odometry, '/pre_filter'),
            message_filters.Subscriber(self, Odometry, '/post_filter')]
            , 5)
        self._time_sync.registerCallback(self.odom_callback)

    def odom_callback(self, pre: Odometry, post: Odometry):
        self._pre_msgs.append(pre)
        self._post_msgs.append(post)
        if len(self._pre_msgs) >= NUM_MESSAGES:
            self.plot_msgs()
            self._pre_msgs.clear()
            self._post_msgs.clear()

    def plot_msgs(self):
        # Convert quaternions to Euler angles
        pre_pose_rpys = [q_to_rpy(pre.pose.pose.orientation) for pre in self._pre_msgs]
        post_pose_rpys = [q_to_rpy(post.pose.pose.orientation) for post in self._post_msgs]

        # Create a figure and 12 subplots, 6 for the pose and 6 for the twist
        fig, ((axpx, axpy, axpz), (axtx, axty, axtz),
              (axproll, axppitch, axpyaw), (axtroll, axtpitch, axtyaw)) = plt.subplots(4, 3)

        # Plot pre-filter values
        axes = [axpx, axpy, axpz, axproll, axppitch, axpyaw]
        valuess = [[msg.pose.pose.position.x for msg in self._pre_msgs],
                   [msg.pose.pose.position.y for msg in self._pre_msgs],
                   [msg.pose.pose.position.z for msg in self._pre_msgs],
                   [rpy[0] for rpy in pre_pose_rpys],
                   [rpy[1] for rpy in pre_pose_rpys],
                   [rpy[2] for rpy in pre_pose_rpys]]
        for ax, values in zip(axes, valuess):
            ax.plot(values, marker='x', ls='', label='pre')

        # Plot post-filter values
        axes = [axpx, axpy, axpz, axtx, axty, axtz, axproll, axppitch, axpyaw, axtroll, axtpitch, axtyaw]
        names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'v roll', 'v pitch', 'v yaw']
        valuess = [[msg.pose.pose.position.x for msg in self._post_msgs],
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
        lims = [(-1.0, 1.0), (-1.0, 1.0), (-3.0, -1.0),
                (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),
                (-4.0, 4.0), (-4.0, 4.0), (-4.0, 4.0),
                (-4.0, 4.0), (-4.0, 4.0), (-4.0, 4.0)]
        for ax, name, values, lim in zip(axes, names, valuess, lims):
            u = statistics.mean(values)
            s = statistics.stdev(values, u)
            ax.set_title('{}, mean {:.3f}, stddev {:.3f}'.format(name, u, s))
            ax.set_ylim(lim)
            ax.set_xticklabels([])
            ax.plot(values, label='post')
            ax.legend()

        # Set figure title
        fig.suptitle('{} pre- and post-filter odometry messages'.format(NUM_MESSAGES))

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
