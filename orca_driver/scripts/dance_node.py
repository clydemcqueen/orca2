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
Generate /control commands to move around a bit.

Usage:
-- ros2 run orca_driver dance_node.py
-- set dance parameter to 'wag', etc.
"""

from typing import Optional

from orca_msgs.msg import Control
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

CMD = 'dance'
CMD_WAG = 'wag'
CMD_STOP = 'stop'

TIMER_PERIOD = 0.1


class DanceNode(Node):

    def __init__(self):
        super().__init__('dance_node')

        self._control_pub = self.create_publisher(Control, '/control', 10)
        self._timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self._cmd = 'stop'
        self._count = 0

        self.declare_parameter(CMD, '')
        self.set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Usage: set dance parameter')
        self.get_logger().info('   cmd wag - roll back / forth')
        self.get_logger().info('   cmd stop - Stop')

    def validate_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name == CMD and param.type_ == Parameter.Type.STRING:
                if param.value == CMD_WAG:
                    self._cmd = CMD_WAG
                    self.get_logger().info('wag')
                    return SetParametersResult(successful=True)
                elif param.value == CMD_STOP:
                    self._cmd = CMD_STOP
                    self.get_logger().info('stop')
                    return SetParametersResult(successful=True)

        self._cmd = CMD_STOP
        return SetParametersResult(successful=False)

    def timer_callback(self):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thruster_pwm.fr_1 = 1500
        msg.thruster_pwm.fl_2 = 1500
        msg.thruster_pwm.rr_3 = 1500
        msg.thruster_pwm.rl_4 = 1500
        msg.thruster_pwm.vr_5 = 1500
        msg.thruster_pwm.vl_6 = 1500

        if self._cmd == CMD_STOP:
            self._control_pub.publish(msg)

        elif self._cmd == CMD_WAG:
            if msg.header.stamp.sec % 2 == 0:
                msg.thruster_pwm.vr_5 = 1540
                msg.thruster_pwm.vl_6 = 1540
            else:
                msg.thruster_pwm.vr_5 = 1460
                msg.thruster_pwm.vl_6 = 1460
            self._control_pub.publish(msg)


def main():
    rclpy.init()

    node = DanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
