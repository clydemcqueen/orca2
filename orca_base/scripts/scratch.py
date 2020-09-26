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
Scratch area, ignore these routines.
"""

from orca_util import rpy_multiply
from math import pi


# Generate the rotations to split sam_map into 3 parts
def calc_split_rotations():
    rpy2 = [-0.9386816250401716, 1.542176293793717, 2.210640377740816]
    rpy3 = [1.266261150380321, -1.556709895928933, -1.269187122240294]
    rpy4 = [-0.3805596437025709, 1.535206497276646, 2.773721462484426]
    rpy5 = [0.09567427005414746, -1.542608504781614, -0.09207529607723448]
    rotate23 = [0, 0, pi * 2. / 3.]
    rotate45 = [0, 0, pi * 4. / 3.]
    print('rpy2', rpy_multiply(rotate23, rpy2))
    print('rpy3', rpy_multiply(rotate23, rpy3))
    print('rpy4', rpy_multiply(rotate45, rpy4))
    print('rpy5', rpy_multiply(rotate45, rpy5))


calc_split_rotations()