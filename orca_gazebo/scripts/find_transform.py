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

"""Find cube transforms that satisfy xform @ input = output --or-- input @ xform = output."""

import math

import numpy as np
import transformations as xf

pi = math.pi
pi2 = math.pi / 2


# Pretty-print Euler [r, p, y]
def e_to_str(e):
    return '[r={: 06.5f}, p={: 06.5f}, y={: 06.5f}]'.format(e[0], e[1], e[2])


# Pretty-print 3x3 matrix
def m_to_str(m: np.ndarray):
    r = m[:3, :3]
    return np.array2string(r, suppress_small=True)


# Generate all possible cube rotations using Euler angles, there will be duplicates
def gen_all_e_rotations():
    for roll in [0, pi2, pi, -pi2]:
        for pitch in [0, pi2, pi, -pi2]:
            for yaw in [0, pi2, pi, -pi2]:
                yield roll, pitch, yaw


# Generate all 24 unique cube rotations
def gen_unique_m_rotations():
    unique = []
    for r in list(gen_all_e_rotations()):
        m = xf.euler_matrix(r[0], r[1], r[2])
        found = False
        for candidate in unique:
            if np.allclose(m, candidate):
                found = True
                break
        if not found:
            unique.append(m)

    assert len(unique) == 24
    return unique


# Try to turn inputs into outputs
def solve(m_xforms, e_inputs, e_outputs):
    print('looking for solution...')
    for m_xform in m_xforms:
        pre_count = 0
        pose_count = 0

        for e_input, e_output in zip(e_inputs, e_outputs):
            m_input = xf.euler_matrix(e_input[0], e_input[1], e_input[2])
            m_output = xf.euler_matrix(e_output[0], e_output[1], e_output[2])
            m_left = m_xform @ m_input
            m_right = m_input @ m_xform

            if np.allclose(m_left, m_output):
                pre_count += 1
            if np.allclose(m_right, m_output):
                pose_count += 1

        if pre_count == len(e_inputs):
            print('found pre multiply solution:')
            print(m_to_str(m_xform))
        if pose_count == len(e_inputs):
            print('found post multiply solution:')
            print(m_to_str(m_xform))


def main():
    m_xforms = gen_unique_m_rotations()
    e_inputs = [
        [0, -1.5707963267948966, 0],
        [0, -1.5707963267948966, -1.5707963267948966],
        [0, -1.5707963267948966, 1.5707963267948966],
        [0, -1.5707963267948966, 3.141592653589793]
    ]
    e_outputs = [
        [1.5707963267948966, 0, -1.5707963267948966],
        [-1.5707963267948966, 3.141592653589793, 0],
        [1.5707963267948966, 0, 0],
        [1.5707963267948966, 0, 1.5707963267948966]
    ]
    solve(m_xforms, e_inputs, e_outputs)


main()
