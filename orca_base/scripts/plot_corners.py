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
Analyze and plot fiducial_vlam_msgs/msg/Observations messages.

Usage:

ros2 run orca_base plot_corners.py
"""

import queue
import statistics
import threading
import time
from typing import List

from fiducial_vlam_msgs.msg import Observations
import matplotlib
from orca_util import seconds
import rclpy
from rclpy.node import Node
import rclpy.time

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

# Turn off flake8 checking for this late import
import matplotlib.pyplot as plt  # noqa: E402,I100

MIN_RANGE = 0.2
NUM_MESSAGES = 1000


# Set ylim to some reasonable values
def set_ylim_with_min_range(ax):
    limits = ax.get_ylim()
    rng = limits[1] - limits[0]

    if rng < MIN_RANGE:
        adj = (MIN_RANGE - rng) / 2.0
        ax.set_ylim(limits[0] - adj, limits[1] + adj)


def plot_value(ax, x_values, y_values, name):
    ax.plot(x_values, y_values)
    ax.set_xticklabels([])
    ax.set_title(name)
    set_ylim_with_min_range(ax)


class Plotter(object):

    def __init__(self, filename):
        self._msgs: List[Observations] = []
        self._filename = filename
        print('collecting messages for {}'.format(self._filename))

    def add_msg(self, msg):
        self._msgs.append(msg)

    def num_messages(self):
        return len(self._msgs)

    def plot(self):
        print('plotting {}'.format(self._filename))
        start_time = time.process_time()

        # Create a figure with 8 subplots
        fig, ((axx0, axx1, axx2, axx3), (axy0, axy1, axy2, axy3)) = plt.subplots(2, 4)
        subplots = [axx0, axx1, axx2, axx3, axy0, axy1, axy2, axy3]
        names = ['x0', 'x1', 'x2', 'x3', 'y0', 'y1', 'y2', 'y3']

        # x axis for all plots == time
        # For most plots all all messages will be plotted
        all_stamps = [seconds(msg.header.stamp) for msg in self._msgs]

        # Warn on a large gap in timestamps -- this shouldn't happen with threading!
        gaps = [second - first for first, second in zip(all_stamps[:-1], all_stamps[1:])]
        largest_gap = max(gaps)
        if largest_gap > 0.1:
            print('WARNING large time gap {:.2f}s'.format(largest_gap))

        all_valuess = [
            [msg.observations[0].x0 for msg in self._msgs],
            [msg.observations[0].x1 for msg in self._msgs],
            [msg.observations[0].x2 for msg in self._msgs],
            [msg.observations[0].x3 for msg in self._msgs],
            [msg.observations[0].y0 for msg in self._msgs],
            [msg.observations[0].y1 for msg in self._msgs],
            [msg.observations[0].y2 for msg in self._msgs],
            [msg.observations[0].y3 for msg in self._msgs]]

        # Plot all corner values
        for subplot, name, all_values in zip(subplots, names, all_valuess):
            plot_value(subplot, all_stamps, all_values, name)

        # Set figure title
        fig.suptitle('{} messages, {}'.format(len(self._msgs), self._filename))

        # Write the PDF file
        plt.savefig(self._filename)

        # Close the figure to reclaim the memory
        plt.close(fig)

        # Write some stats as well
        means = [statistics.mean(all_values) for all_values in all_valuess]
        stdevs = [statistics.stdev(all_values, mean) for all_values, mean in
                  zip(all_valuess, means)]
        mean_stdev = statistics.mean(stdevs)
        print('means =', means)
        print('stdevs =', stdevs)
        print('mean stdev =', mean_stdev)

        stop_time = time.process_time()
        print('finished {}, elapsed time {:.2f}s'.format(self._filename, stop_time - start_time))
        print()


def consumer(q: queue.Queue):
    while True:
        plotter: Plotter = q.get()
        plotter.plot()


class PlotCornersNode(Node):

    def __init__(self):
        super().__init__('plot_control')
        self._control_sub = self.create_subscription(Observations, '/fiducial_observations',
                                                     self.obs_callback, 10)

        self._plot = None
        self._prev_status = None

        # Use the producer-consumer model
        # We are the producer -- we collect messages in self._plot, then add it to the queue
        self._q = queue.Queue()

        # The consumer takes Plotter objects off the queue and creates the plot
        self._thread = threading.Thread(target=consumer, args=(self._q,))
        self._thread.start()

    def obs_callback(self, msg: Observations):
        # curr_status = PlannerStatus(msg)

        # Bootstrap
        if self._plot is None:
            self._plot = Plotter('plot_corners.pdf')

        # Add message to plotter
        self._plot.add_msg(msg)

        # Plot when when we hit message target
        if self._plot.num_messages() >= NUM_MESSAGES:
            self._q.put(self._plot)
            self._plot = Plotter('plot_corners.pdf')


def main():
    print('backend is', plt.get_backend())

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [24., 12.]

    rclpy.init()
    node = PlotCornersNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
        # TODO notify consumer thread for a clean shutdown -- today you need to hit ctrl-c twice
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
