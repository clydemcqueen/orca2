#!/usr/bin/env python3

# This is a simple ROS wrapper for the BlueRobotics Bar30 sensor
# https://github.com/bluerobotics/ms5837-python/blob/master/ms5837.py must be in the Python path

import time

import ms5837
import rclpy
from builtin_interfaces.msg import Time
from orca_msgs.msg import Barometer
from rclpy.node import Node

DEPTH_STDDEV = 0.01


def now() -> Time:
    """Return builtin_interfaces.msg.Time object with the current CPU time"""
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


class Bar30Node(Node):

    def __init__(self):
        super().__init__('bar30_node')

        # Connect to the Bar30 sensor
        # RPi is bus 1, UP Board is bus 5
        self.get_logger().info("connecting to Bar30...")
        self._sensor = ms5837.MS5837_30BA(5)
        if not self._sensor.init():
            self.get_logger().fatal("can't initialize Bar30")
            exit(1)
        self.get_logger().info("connected to Bar30")
        self._sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)  # TODO verify that this doesn't matter

        # Publish at 10Hz
        self._baro_pub = self.create_publisher(Barometer, '/barometer', 5)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self._sensor.read():
            msg = Barometer()
            msg.header.stamp = now()
            msg.pressure = self._sensor.pressure() * 100.0  # Pascals
            msg.temperature = self._sensor.temperature()  # Celsius
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
