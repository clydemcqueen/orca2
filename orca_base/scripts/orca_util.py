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
Utilities.

Usage:

import orca_util.py
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
import transformations as xf


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def norm_angle(x):
    x = x % (2 * math.pi)  # force in range [0, 2 pi)
    if x > math.pi:  # move to [-pi, pi)
        x -= 2 * math.pi
    return x


def get_yaw(q: Quaternion) -> float:
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy[2]


# [r, p, y] to geometry_msgs.msg.Quaternion
def rpy_to_q(r, p, y) -> Quaternion:
    matrix = xf.euler_matrix(r, p, y, axes='sxyz')
    q = xf.quaternion_from_matrix(matrix)
    return Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])  # Order is w, x, y, z


# geometry_msgs.msg.Quaternion to [r, p, y]
def q_to_rpy(q: Quaternion) -> []:
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy


# Multiple 2 geometry_msgs.msg.Quaternions
def q_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    result = xf.quaternion_multiply([left.w, left.x, left.y, left.z],
                                    [right.w, right.x, right.y, right.z])
    return Quaternion(w=result[0], x=result[1], y=result[2], z=result[3])


# Multiple 2 rpys
def rpy_multiply(left: [], right: []) -> []:
    q_result = q_multiply(rpy_to_q(*left), rpy_to_q(*right))
    return q_to_rpy(q_result)


def get_quaternion(yaw: float) -> Quaternion:
    m = xf.euler_matrix(0, 0, yaw)
    q = xf.quaternion_from_matrix(m)
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])  # Order is w, x, y, z


# Set ylim to some reasonable values
def set_ylim_with_min_range(ax, min_range=0.2):
    limits = ax.get_ylim()
    ylim_range = limits[1] - limits[0]

    if ylim_range < min_range:
        adj = (min_range - ylim_range) / 2.0
        ax.set_ylim(limits[0] - adj, limits[1] + adj)
