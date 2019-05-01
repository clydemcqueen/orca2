#!/usr/bin/env python

# This is a simple ROS wrapper for the BlueRobotics Bar30 sensor
# https://github.com/bluerobotics/ms5837-python/blob/master/ms5837.py must be in the Python path

import rclpy
from rclpy.node import Node

from orca_msgs.msg import Barometer
import ms5837


class Bar30Node(Node):

    def __init__(self):
        super().__init__('bar30_node')

        # Connect to the Bar30 sensor on I2C bus 1
        self.get_logger().info("connecting to Bar30...")
        self._sensor = ms5837.MS5837_30BA(1)
        if not self._sensor.init():
            self.get_logger().fatal("can't initialize Bar30")
            exit(1)
        self.get_logger().info("connected to Bar30")
        self._sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)

        # Publish at 10Hz
        self._baro_pub = self.create_publisher(Barometer, '/barometer')
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self._sensor.read():
            msg = Barometer()
            msg.header.stamp = self.get_clock().now()
            msg.pressure = self._sensor.pressure() * 100.0  # Pascals
            msg.temperature = self._sensor.temperature()    # Celsius
            msg.depth = self._sensor.depth()                # meters
            self._baro_pub.publish(msg)
        else:
            self.get_logger().error("can't read Bar30")


def main(args=None):
    rclpy.init(args=args)
    node = Bar30Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
