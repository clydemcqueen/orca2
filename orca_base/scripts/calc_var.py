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
Simple node to calculate variance.

Usage:

-- update the code to sample the appropriate message
-- ros2 run orca_base calc_var.py
"""

import statistics

from orca_msgs.msg import Depth
import rclpy
from rclpy.node import Node
import rclpy.time

# How many messages (measurements) to sample?
# Run `ros2 bag info my_bag` to count messages in a ROS2 bag
NUM_MEASUREMENTS = 1500


class CalcVarNode(Node):

    def __init__(self):
        super().__init__('calculate_variance')
        self._control_sub = self.create_subscription(Depth, '/depth', self.callback, 10)
        self._measurements = []

    def callback(self, msg: Depth):
        self._measurements.append(msg._z)

        if 0 < NUM_MEASUREMENTS <= len(self._measurements):
            mean = statistics.mean(self._measurements)
            variance = statistics.variance(self._measurements, mean)
            self.get_logger().info('n: {}, mean: {}, variance: {}'.format(
                len(self._measurements), mean, variance))
            exit()


def main():
    rclpy.init()
    node = CalcVarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
