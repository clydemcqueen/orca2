#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu

import numpy as np

import kf

# Estimate a local pose

# -- position (hidden)
# -- velocity (hidden)
# -- acceleration (measured)

G = 9.80665     # Gravitational constant
dt = 1. / 50    # TODO get dt from message


class LocalPose(Node):

    def __init__(self):
        super().__init__('local_pose')

        self._kf = kf.KalmanFilter(state_dim=9, measurement_dim=3)

        # TODO drag ???
        dt2 = 0.5 * dt * dt
        self._kf.F = np.array([
            [1, 0, 0,    dt, 0, 0,    dt2, 0, 0],   # x
            [0, 1, 0,    0, dt, 0,    0, dt2, 0],   # y
            [0, 0, 1,    0, 0, dt,    0, 0, dt2],   # z

            [0, 0, 0,    1, 0, 0,     dt, 0, 0],    # x'
            [0, 0, 0,    0, 1, 0,     0, dt, 0],    # y'
            [0, 0, 0,    0, 0, 1,     0, 0, dt],    # z'

            [0, 0, 0,    0, 0, 0,     1, 0, 0],     # x''
            [0, 0, 0,    0, 0, 0,     0, 1, 0],     # y''
            [0, 0, 0,    0, 0, 0,     0, 0, 1],     # z''
        ])
        print('F\n', self._kf.F)

        # Process noise
        # TODO look at this more carefully
        # Q_var = 0.0001
        # Q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_var)
        self._kf.Q = np.eye(9) * 0.0001
        # self.kf.Q = np.array([[2.5e-05, 5.0e-05, 5.0e-05], [5.0e-05, 1.0e-04, 1.0e-04], [5.0e-05, 1.0e-04, 1.0e-04]])
        print('Q\n', self._kf.Q)

        # Measurement matrix, translates state into measurement space
        self._kf.H = np.array([
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ])

        print('H\n', self._kf.H)

        # Measurement noise
        self._R = np.eye(3) * 0.003 * 0.003
        print('R\n', self._R)

        # Initialize path message
        self._path = Path()
        self._path.header.frame_id = 'odom'

        # Set up publications and subscriptions
        self._path_pub = self.create_publisher(Path, '/orca_base/python_estimated')
        self.create_subscription(Imu, '/imu/data', self._imu_callback)

    def _imu_callback(self, data):
        # TODO rotate lin acc into place

        # Estimate position
        self._kf.predict()
        z = np.array([[data.linear_acceleration.x], [data.linear_acceleration.y], [data.linear_acceleration.z - G]])
        self._kf.update(z, self._R)

        # Update path message and publish
        pose = PoseStamped()
        pose.header.stamp = data.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self._kf.x[0]
        pose.pose.position.y = self._kf.x[1]
        pose.pose.position.z = self._kf.x[2]
        pose.pose.orientation = data.orientation
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    np.set_printoptions(precision=4, suppress=True)
    node = LocalPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
