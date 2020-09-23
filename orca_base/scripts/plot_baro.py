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
Analyze and plot orca_msgs/msg/Baromter messages.

Usage:
ros2 run orca_base plot_baro.py
ros2 run orca_base plot_baro.py 200
"""
import statistics
import sys
from typing import List

import matplotlib
from orca_msgs.msg import Barometer, Control
from orca_util import seconds, set_ylim_with_min_range
import rclpy
from rclpy.node import Node
import rclpy.time

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

# Turn off flake8 checking for this late import
import matplotlib.pyplot as plt  # noqa: E402,I100


# Y axis is pressure in Pascals, but these are hard to interpret. For convenience,
# convert to depth using default values for core constants, and set the target to 0.
# This allows me to set a reasonable ylim.
def depth(delta_pressure):
    return -delta_pressure / (997.0 * 9.8)


class PlotBaroNode(Node):

    def __init__(self, num_messages):
        super().__init__('plot_baro')
        self._baro_msgs: List[Barometer] = []
        self._filtered_baro_msgs: List[Barometer] = []
        self._control_msgs: List[Control] = []
        self._baro_sub = self.create_subscription(Barometer, '/barometer',
                                                  self.baro_callback, 10)
        self._filtered_baro_sub = self.create_subscription(Barometer, '/filtered_barometer',
                                                           self.filtered_baro_callback, 10)
        self._control_sub = self.create_subscription(Control, '/control',
                                                     self.control_callback, 10)
        self._num_messages = num_messages

    def baro_callback(self, msg: Barometer):
        self._baro_msgs.append(msg)
        if len(self._baro_msgs) >= self._num_messages:
            self.plot_msgs()
            self._baro_msgs.clear()
            self._filtered_baro_msgs.clear()
            self._control_msgs.clear()

    def filtered_baro_callback(self, msg: Barometer):
        self._filtered_baro_msgs.append(msg)

    def control_callback(self, msg: Control):
        self._control_msgs.append(msg)

    def plot_msgs(self):
        # Create a figure and 1 subplot
        fig, (plot_pressure) = plt.subplots(1)

        # X axis is seconds since the 1st message in the series
        start_time = seconds(self._baro_msgs[0].header.stamp)

        # For convenience Y axis is depth... but we don't have absolute depth w/o
        # a calibrated air pressure. So, assume the mean filtered pressure is depth==0.
        reference = statistics.mean([msg.pressure for msg in self._filtered_baro_msgs])
        plot_pressure.set_title('Black is mean filtered pressure, magenta is target')

        depth_x = [seconds(msg.header.stamp) - start_time for msg in self._baro_msgs]
        depth_y = [depth(msg.pressure - reference) for msg in self._baro_msgs]

        filt_depth_x = [seconds(msg.header.stamp) - start_time for msg in self._filtered_baro_msgs]
        filt_depth_y = [depth(msg.pressure - reference) for msg in self._filtered_baro_msgs]

        plot_pressure.plot(depth_x, depth_y, label='depth')
        plot_pressure.plot(filt_depth_x, filt_depth_y, label='filtered depth')

        # Draw line at y=0
        plot_pressure.axhline(y=0, color='k')

        # Draw line at the target depth
        if len(self._control_msgs) > 0:
            plot_pressure.axhline(y=depth(self._control_msgs[0].target_pressure - reference),
                                  color='m')

        plot_pressure.legend()

        # Y limits +- 1cm
        plot_pressure.set_ylim(-0.2, 0.2)
        # set_ylim_with_min_range(plot_pressure, 0.15)

        # Set figure title
        fig.suptitle('{} messages'.format(self._num_messages))

        # [Over]write PDF to disk
        plt.savefig('plot_baro.pdf')

        # Close the figure to reclaim the memory
        plt.close(fig)


def main():
    print('backend is', plt.get_backend())

    num_messages = 200
    if len(sys.argv) > 1:
        num_messages = int(sys.argv[1])
    print('num messages =', num_messages)

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [8., 4.]

    rclpy.init()
    node = PlotBaroNode(num_messages)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
