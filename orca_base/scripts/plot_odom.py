#!/usr/bin/env python3

"""
Analyze and plot nav_msgs/msg/Odometry messages

Usage: ros2 run orca_base plot_odom.py /odom:=/forward_camera/odom
"""

import statistics
from typing import List

import matplotlib
import matplotlib.pyplot as plt
import rclpy
import transformations as xf
from nav_msgs.msg import Odometry
from rclpy.node import Node

NUM_MESSAGES = 200


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


class PlotOdomNode(Node):

    def __init__(self):
        super().__init__('plot_odom')
        self._odom_msgs: List[Odometry] = []
        self._odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        self._odom_msgs.append(msg)
        if len(self._odom_msgs) >= NUM_MESSAGES:
            self.plot_msgs()
            self._odom_msgs.clear()

    def plot_msgs(self):
        # Create a figure and 8 subplots, one for each dimension plus a blank
        fig, ((axpx, axpy, axpz, blank), (axow, axox, axoy, axoz)) = plt.subplots(2, 4)

        # Get RPY
        # msgs_rpy = [q_to_rpy(msg.pose.pose.orientation) for msg in self._odom_msgs]

        # Plot
        axes = [axpx, axpy, axpz, axow, axox, axoy, axoz]
        names = ['position.x', 'position.y', 'position.z',
                 'orientation.w', 'orientation.x', 'orientation.y', 'orientation.z']
        valuess = [[msg.pose.pose.position.x for msg in self._odom_msgs],
                   [msg.pose.pose.position.y for msg in self._odom_msgs],
                   [msg.pose.pose.position.z for msg in self._odom_msgs],
                   [msg.pose.pose.orientation.w for msg in self._odom_msgs],
                   [msg.pose.pose.orientation.x for msg in self._odom_msgs],
                   [msg.pose.pose.orientation.y for msg in self._odom_msgs],
                   [msg.pose.pose.orientation.z for msg in self._odom_msgs]]
        lims = [(-2.0, 2.0), (-2.0, 2.0), (-4.0, 0.0),
                (-1.1, 1.1), (-1.1, 1.1), (-1.1, 1.1), (-1.1, 1.1)]
        for ax, name, values, lim in zip(axes, names, valuess, lims):
            u = statistics.mean(values)
            s = statistics.stdev(values, u)
            ax.set_title('{}, mean {:.3f}, stddev {:.3f}'.format(name, u, s))
            ax.set_ylim(lim)
            ax.set_xticklabels([])
            ax.plot(values)

        # Set figure title
        fig.suptitle('{} odometry messages'.format(NUM_MESSAGES))

        # [Over]write PDF to disk
        plt.savefig('plot_odom.pdf')

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
    plt.rcParams['figure.figsize'] = [21., 7.]

    rclpy.init(args=args)
    node = PlotOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
