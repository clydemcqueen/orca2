#!/usr/bin/env python3

"""
Utilities

Usage:

import orca_util.py
"""

import math

import transformations as xf
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def normalize_angle(x):
    x = x % (2 * math.pi)  # force in range [0, 2 pi)
    if x > math.pi:  # move to [-pi, pi)
        x -= 2 * math.pi
    return x


def get_yaw(q: Quaternion) -> float:
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy[2]


def q_to_rpy(q):
    m = xf.quaternion_matrix([q.w, q.x, q.y, q.z])  # Order is w, x, y, z
    rpy = xf.euler_from_matrix(m)
    return rpy
