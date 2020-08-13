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
Compute Normalized Estimated Error Squared (NEES).

See https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/, chapter 08

Only works in simulations, and only useful if we're running a filter
"""

from typing import List, Optional

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import numpy as np
from orca_msgs.msg import FiducialPoseStamped
from scipy.linalg import inv
import transformations as xf


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


def normalize_angle(x):
    x = x % (2 * np.pi)  # force in range [0, 2 pi)
    if x > np.pi:  # move to [-pi, pi)
        x -= 2 * np.pi
    return x


class State(object):
    """Filter state."""

    def __init__(self, s=0., x=np.zeros(6), P=np.identity(6)):
        self._s = s  # Timestamp, in seconds
        self._x = x  # x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw
        self._P = P

    def s(self) -> float:
        return self._s

    def __str__(self):
        return 's: ' + str(self._s) + '\nx: ' + str(self._x) + '\n'

    @classmethod
    def from_fp(cls, msg: FiducialPoseStamped):
        x = np.array([msg.fp.pose.pose.position.x,
                      msg.fp.pose.pose.position.y,
                      msg.fp.pose.pose.position.z,
                      *q_to_rpy(msg.fp.pose.pose.orientation)])
        P = np.zeros((6, 6))
        P[0:6, 0:6] = msg.fp.pose.covariance.reshape((6, 6))  # Better way to do this?
        return State(seconds(msg.header.stamp), x, P)

    @classmethod
    def from_odometry(cls, msg: Odometry):
        x = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z,
                      *q_to_rpy(msg.pose.pose.orientation)])
        P = np.zeros((6, 6))
        P[0:6, 0:6] = msg.pose.covariance.reshape((6, 6))  # Better way to do this?
        return State(seconds(msg.header.stamp), x, P)

    @classmethod
    def interpolate(cls, a: 'State', b: 'State', s: float) -> Optional['State']:
        """Interpolate between a and b."""
        epsilon = 0.001
        if abs(a._s - s) < epsilon:
            return a
        elif abs(b._s - s) < epsilon:
            return b
        elif a._s < s < b._s:
            denom = (b._s - a._s)
            a_factor = (b._s - s) / denom
            b_factor = (s - a._s) / denom
            x = a._x * a_factor + b._x * b_factor
            P = a._P * a_factor + b._P * b_factor

            # Normalize the angles
            x[3] = normalize_angle(x[3])
            x[4] = normalize_angle(x[4])
            x[5] = normalize_angle(x[5])

            return State(s, x, P)
        else:
            return None

    @classmethod
    def nees(cls, truth: 'State', estimate: 'State') -> float:
        """Calc NEES."""
        error_x = truth._x - estimate._x

        # Normalize the angles
        error_x[3] = normalize_angle(error_x[3])
        error_x[4] = normalize_angle(error_x[4])
        error_x[5] = normalize_angle(error_x[5])

        return np.dot(error_x.T, inv(estimate._P)).dot(error_x)


def nees(e_msgs: List[FiducialPoseStamped], gt_msgs: List[Odometry]) -> List[float]:
    """
    Given lists of estimated values and true values, calculate a list of NEES values.

    Interpolate between the true values as necessary. The lists must be sorted by time.
    """
    estimates = []
    for e_msg in e_msgs:
        estimates.append(State.from_fp(e_msg))

    truths = []
    for gt_msg in gt_msgs:
        truths.append(State.from_odometry(gt_msg))

    results = []
    for estimate in estimates:
        for truth1, truth2 in zip(truths, truths[1:]):
            truth = State.interpolate(truth1, truth2, estimate.s())
            if truth is not None:
                results.append(State.nees(truth, estimate))
                break

    return results


def main():
    print('Test nees_fp.py')

    a = State(0., np.ones(6) * 10.)
    b = State(1., np.ones(6) * 20.)
    c = State(2., np.ones(6) * 20.5)
    print('a\n', a)
    print('b\n', b)
    print('c\n', c)
    print('interpolate 0.1 (between a and b)\n', State.interpolate(a, b, 0.1))
    print('nees a, b', State.nees(a, b))
    print('nees b, c', State.nees(b, c))


if __name__ == '__main__':
    main()
