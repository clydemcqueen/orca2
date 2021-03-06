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
Analyze and plot orca_msgs/msg/Control messages.

Usage:
ros2 run orca_base plot_control.py
ros2 run orca_base plot_control.py 200

Updating plots is tricky.

Method 1 (failed):
-- turn interactive mode on using plt.ion()
-- call plt.pause(0.01) to allow/force an update

Unfortunately, plt.pause() appears to call activateWindow() on the Qt5Agg backend.
This brings the Qt5Agg window to the front and grabs focus, interrupting anything else you're
doing. This is annoying.

Method 2 (current):
-- use the PDF backend to write plots to PDF files
-- view the files using `evince fig.pdf`

Evince watches the file and reloads whenever it changes. The downside is that the plot is a static
image, so resizing the window doesn't do what you expect.

Perhaps worth investigating:
-- custom mypause function that doesn't call activateWindow()
-- matplotlib animations
"""

import sys
from typing import List

import matplotlib
from orca_msgs.msg import Control
import rclpy
from rclpy.node import Node
import rclpy.time

# Set backend before importing matplotlib.pyplot
matplotlib.use('pdf')

# Turn off flake8 checking for this late import
import matplotlib.pyplot as plt  # noqa: E402,I100


def cost_function(pos_neg, off_on):
    """
    Cost function for a sequence of control messages, lower is better.

    :param pos_neg: count of forward<->reverse transitions, cost=10
    :param off_on: count of off->on transitions, cost=5
    :return: cost
    """
    return 10 * pos_neg + 5 * off_on


class PlotControlNode(Node):

    def __init__(self, num_messages):
        super().__init__('plot_control')
        self._control_msgs: List[Control] = []
        self._control_sub = self.create_subscription(Control, '/control',
                                                     self.control_callback, 10)
        self._num_messages = num_messages

    def control_callback(self, msg: Control):
        self._control_msgs.append(msg)
        if len(self._control_msgs) >= self._num_messages:
            self.plot_msgs()
            self._control_msgs.clear()

    def plot_msgs(self):
        # Create a figure and 16 subplots:
        # 4 for efforts in the body frame
        # 6 for thruster PMW values
        # 1 for odom lag
        # 1 for dt
        fig, ((axff, axfs, axfv, axfw), (axt0, axt1, axt2, axt3),
              (axt4, axt5, axol, axdt)) = plt.subplots(3, 4)

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
            ax.set_ylim(-0.3, 0.3)
            ax.set_xticklabels([])
            ax.plot(values)

        # Plot thruster PWM values
        fr_1 = [msg.thruster_pwm.fr_1 for msg in self._control_msgs]
        fl_2 = [msg.thruster_pwm.fl_2 for msg in self._control_msgs]
        rr_3 = [msg.thruster_pwm.rr_3 for msg in self._control_msgs]
        rl_4 = [msg.thruster_pwm.rl_4 for msg in self._control_msgs]
        vr_5 = [msg.thruster_pwm.vr_5 for msg in self._control_msgs]
        vl_6 = [msg.thruster_pwm.vl_6 for msg in self._control_msgs]

        pwm_axes = [axt0, axt1, axt2, axt3, axt4, axt5]
        pwm_valuess = [fr_1, fl_2, rr_3, rl_4, vr_5, vl_6]
        pwm_names = ['fr_1', 'fl_2', 'rr_3', 'rl_4', 'vr_5', 'vl_6']
        total_cost = 0
        for ax, values, name in zip(pwm_axes, pwm_valuess, pwm_names):
            # Measure fitness
            pos_neg, off_on = 0, 0
            for c, n in zip(values, values[1:]):
                if c > 1500 > n:
                    pos_neg += 1
                elif c < 1500 < n:
                    pos_neg += 1
                elif c == 1500 and n != 1500:
                    off_on += 1
            cost = cost_function(pos_neg, off_on)
            total_cost += cost

            ax.set_title('{}: pos_neg={}, off_on={}, cost={}'.format(name, pos_neg, off_on, cost))
            ax.set_ylim(1400, 1600)
            ax.set_xticklabels([])
            ax.plot(values)

        # Plot odom lag. These values depend on the node's estimate of "now" which isn't all that
        # good during a simulation.
        odom_lag_values = [msg.odom_lag for msg in self._control_msgs]
        axol.set_title('odom lag, ave={0:.3f}'.format(sum(odom_lag_values) / len(odom_lag_values)))
        axol.set_ylim(-0.1, 0.1)
        axol.set_xticklabels([])
        axol.plot(odom_lag_values)

        # Plot dt
        # Also might be dicey in a simulation
        dt_values = []
        prev_t = rclpy.time.Time.from_msg(self._control_msgs[0].header.stamp)
        for msg in self._control_msgs[1:]:
            msg_t = rclpy.time.Time.from_msg(msg.header.stamp)
            dt_values.append((msg_t - prev_t).nanoseconds / 1000000)
            prev_t = msg_t
        axdt.set_title('dt (ms), ave={0:.3f}'.format(sum(dt_values) / len(dt_values)))
        axdt.set_ylim(-1, 200)
        axdt.set_xticklabels([])
        axdt.plot(dt_values)

        # Set figure title
        fig.suptitle('{} messages, total cost={}'.format(self._num_messages, total_cost))

        # [Over]write PDF to disk
        plt.savefig('plot_control.pdf')

        # Close the figure to reclaim the memory
        plt.close(fig)


def main():
    print('backend is', plt.get_backend())

    num_messages = 100
    if len(sys.argv) > 1:
        num_messages = int(sys.argv[1])
    print('num messages =', num_messages)

    # Set figure size (inches)
    plt.rcParams['figure.figsize'] = [24., 12.]

    rclpy.init()
    node = PlotControlNode(num_messages)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
