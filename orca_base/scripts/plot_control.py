#!/usr/bin/env python3

"""
Analyze and plot orca_msg/Control messages
"""

from typing import List

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from orca_msgs.msg import Control
from rclpy.node import Node


def calc_usage(fwd_rev, off_on, on):
    """
    Thruster usage function for a sequence of control messages, high usage is presumably bad
    :param fwd_rev: count of fwd<->rev transitions, cost=10
    :param off_on: count of off->on transitions, cost=5
    :param on: count of on states, cost=1
    :return: goodness
    """
    return 10 * fwd_rev + 5 * off_on + on


class PlotControlNode(Node):

    def __init__(self):
        super().__init__('plot_control')
        self._control_msgs: List[Control] = []
        self._control_sub = self.create_subscription(Control, '/control', self.control_callback, 10)
        print('listening for /control messages')

    def control_callback(self, msg: Control):
        self._control_msgs.append(msg)
        if len(self._control_msgs) >= 100:
            self.plot_msgs()
            self._control_msgs.clear()

    def plot_msgs(self):
        # Create a figure and 6 subplots, one for each thruster
        fig, ((ax0, ax1), (ax2, ax3), (ax4, ax5)) = plt.subplots(3, 2)

        ax = [ax0, ax1, ax2, ax3, ax4, ax5]
        total_usage = 0
        for i in range(6):
            # Full PWM range is [1100, 1900] but the AUV typically doesn't use the full range
            ax[i].set_ylim(1400, 1600)

            # Plot thruster PWM values
            ax[i].plot([msg.thruster_pwm[i] for msg in self._control_msgs])

            # Compute usage function
            fwd_rev, off_on, on = 0, 0, 0
            for c, n in zip(self._control_msgs, self._control_msgs[1:]):
                if c.thruster_pwm[i] > 1500 > n.thruster_pwm[i]:
                    fwd_rev += 1
                elif c.thruster_pwm[i] < 1500 < n.thruster_pwm[i]:
                    fwd_rev += 1
                elif c.thruster_pwm[i] == 1500 and n.thruster_pwm[i] != 1500:
                    off_on += 1
                if c.thruster_pwm[i] != 1500:
                    on += 1
            thruster_usage = calc_usage(fwd_rev, off_on, on)
            total_usage += thruster_usage

            # Set subplot title
            ax[i].set_title(
                'T{}: fwd_rev={}, off_on={}, on={}, usage={}'.format(i, fwd_rev, off_on, on, thruster_usage))

        # Set figure title
        fig.suptitle('total usage={}'.format(total_usage))
        print(total_usage)

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
    plt.rcParams['figure.figsize'] = [12., 12.]

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
