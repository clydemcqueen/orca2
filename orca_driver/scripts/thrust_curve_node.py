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
Generate /control commands to generate a thrust curve.

Usage:
-- set up a jig to measure force
-- ros2 run orca_driver thrust_curve_node.py
-- set cmd parameter to 'start' to start, 'inc' to increment, 'stop' to stop
-- measure thrust force at each stage
"""

from typing import Optional

from orca_msgs.msg import Control
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

CMD = 'cmd'
CMD_START = 'start'
CMD_INC = 'inc'
CMD_DEC = 'dec'
CMD_STOP = 'stop'

# Ramp up / down to target pwm to avoid jerks
TIMER_PERIOD = 0.1
RAMP_INC = 1


class ThrustCurveNode(Node):

    def __init__(self, start_pwm, inc_pwm):
        super().__init__('thrust_curve_node')

        if start_pwm <= 1500:
            self.get_logger().warn('start_pwm must be > 1500, starting at 1530')
            start_pwm = 1530

        if inc_pwm < 1 or inc_pwm > 100:
            self.get_logger().warn('inc_pwm must be 1-100, increment by 10')
            inc_pwm = 10

        self._start_pwm = start_pwm
        self._inc_pwm = inc_pwm

        self._curr_pwm = 1500
        self._target_pwm = 1500

        self._control_pub = self.create_publisher(Control, '/control', 10)
        self._timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.declare_parameter(CMD, '')
        self.set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Usage: set cmd parameter')
        self.get_logger().info('   cmd start - Start')
        self.get_logger().info('   cmd inc - Increment')
        self.get_logger().info('   cmd dec - Decrement')
        self.get_logger().info('   cmd stop - Stop')

    def validate_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name == CMD and param.type_ == Parameter.Type.STRING:
                if param.value == CMD_START:
                    self._target_pwm = self._start_pwm
                    self.get_logger().info('start, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_INC:
                    self._target_pwm = self._target_pwm + self._inc_pwm
                    self.get_logger().info('increment, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_DEC:
                    self._target_pwm = self._target_pwm - self._inc_pwm
                    self.get_logger().info('decrement, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_STOP:
                    self._target_pwm = 1500
                    self.get_logger().info('stop, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)

        return SetParametersResult(successful=False)

    def timer_callback(self):
        # Ramp to target pwm
        if self._curr_pwm < self._target_pwm:
            self._curr_pwm = self._curr_pwm + RAMP_INC
        elif self._curr_pwm > self._target_pwm:
            self._curr_pwm = self._curr_pwm - RAMP_INC

        # Send message
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thruster_pwm.fr_1 = self._curr_pwm
        msg.thruster_pwm.fl_2 = self._curr_pwm
        msg.thruster_pwm.rr_3 = self._curr_pwm
        msg.thruster_pwm.rl_4 = self._curr_pwm
        msg.thruster_pwm.vr_5 = 1500
        msg.thruster_pwm.vl_6 = 1500
        self._control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ThrustCurveNode(start_pwm=1600, inc_pwm=5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
