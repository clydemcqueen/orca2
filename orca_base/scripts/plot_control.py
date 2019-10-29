#!/usr/bin/env python3

"""
Analyze and plot orca_msgs/msg/Control messages

Usage: ros2 run orca_base plot_control.py
"""

from typing import List

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from orca_msgs.msg import Control
from rclpy.node import Node

NUM_MESSAGES = 200


def cost_function(pos_neg, off_on, on):
    """
    Cost function for a sequence of control messages, lower is better
    :param pos_neg: count of forward<->reverse transitions, cost=10
    :param off_on: count of off->on transitions, cost=5
    :param on: count of on states, cost=0
    :return: cost
    """
    return 10 * pos_neg + 5 * off_on # + on


class PlotControlNode(Node):

    def __init__(self):
        super().__init__('plot_control')
        self._control_msgs: List[Control] = []
        self._control_sub = self.create_subscription(Control, '/control', self.control_callback, 10)

    def control_callback(self, msg: Control):
        self._control_msgs.append(msg)
        if len(self._control_msgs) >= NUM_MESSAGES:
            self.plot_msgs()
            self._control_msgs.clear()

    def plot_msgs(self):
        # Create a figure and 16 subplots:
        # 4 for error in the world frame
        # 4 for efforts in the body frame
        # 6 for thruster PMW values
        # 1 for odom lag
        # 1 blank
        fig, ((axex, axey, axez, axew), (axff, axfs, axfv, axfw), (axt0, axt1, axt2, axt3),
              (axt4, axt5, axol, blank)) = plt.subplots(4, 4)

        # Plot error
        error_axes = [axex, axey, axez, axew]
        error_names = ['error x', 'error y', 'error z', 'error yaw']
        error_values = [[msg.error.x for msg in self._control_msgs],
                        [msg.error.y for msg in self._control_msgs],
                        [msg.error.z for msg in self._control_msgs],
                        [msg.error.yaw for msg in self._control_msgs]]
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
            ax.set_ylim(-0.3, 0.3)
            ax.set_xticklabels([])
            ax.plot(values)

        # Plot thruster PWM values
        pwm_axes = [axt0, axt1, axt2, axt3, axt4, axt5]
        total_cost = 0
        for i, ax in zip(range(6), pwm_axes):
            pwm_values = [msg.thruster_pwm[i] for msg in self._control_msgs]

            # Measure fitness
            pos_neg, off_on, on = 0, 0, 0
            for c, n in zip(pwm_values, pwm_values[1:]):
                if c > 1500 > n:
                    pos_neg += 1
                elif c < 1500 < n:
                    pos_neg += 1
                elif c == 1500 and n != 1500:
                    off_on += 1
                if c != 1500:
                    on += 1
            cost = cost_function(pos_neg, off_on, on)
            total_cost += cost

            ax.set_title('T{}: pos_neg={}, off_on={}, on={}, cost={}'.format(i, pos_neg, off_on, on, cost))
            ax.set_ylim(1400, 1600)
            ax.set_xticklabels([])
            ax.plot(pwm_values)

        # Plot odom lag
        # These values depend on the node's estimate of "now" which isn't all that good during a simulation
        odom_lag_values = [msg.odom_lag for msg in self._control_msgs]
        axol.set_title('odom lag, ave={0:.3f}'.format(sum(odom_lag_values) / len(odom_lag_values)))
        axol.set_ylim(-0.1, 0.1)
        axol.set_xticklabels([])
        axol.plot(odom_lag_values)

        # Set figure title
        fig.suptitle('{} messages, total cost={}'.format(NUM_MESSAGES, total_cost))

        # [Over]write PDF to disk
        plt.savefig('plot_control.pdf')

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
    plt.rcParams['figure.figsize'] = [24., 12.]

    rclpy.init(args=args)
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
