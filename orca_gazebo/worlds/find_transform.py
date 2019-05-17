#!/usr/bin/env python

"""
Given a set of input and output orientations, find all "right-angle" transforms that satisfy
    xform @ input => output  --or--
    input @ xform => output
"""

import math
import numpy as np
import transformations as xf

pi = math.pi
pi2 = math.pi/2


# Pretty-print Euler [r, p, y]
def e_to_str(e):
    return '[r={: 06.5f}, p={: 06.5f}, y={: 06.5f}]'.format(e[0], e[1], e[2])


# Pretty-print matrix
def m_to_str(m: np.ndarray):
    r = m[:3, :3]
    return np.array2string(r, suppress_small=True)


# Generate all possible cube rotations using Euler angles, there will be duplicates
def gen_all_euler_rotations():
    for roll in [0, pi2, pi, -pi2]:
        for pitch in [0, pi2, pi, -pi2]:
            for yaw in [0, pi2, pi, -pi2]:
                yield roll, pitch, yaw


# Generate all 24 unique cube rotations
def gen_unique_m_rotations():
    unique = []
    for r in list(gen_all_euler_rotations()):
        m = xf.euler_matrix(r[0], r[1], r[2])
        hit = False
        for candidate in unique:
            if np.allclose(m, candidate):
                hit = True
                break
        if not hit:
            unique.append(m)

    assert len(unique) == 24
    return unique


unique_m_rotations = gen_unique_m_rotations()

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

# Try to turn inputs into outputs
print('solve:')
for m_xform in unique_m_rotations:
    for e_input, e_output in zip(e_inputs, e_outputs):
        m_input = xf.euler_matrix(e_input[0], e_input[1], e_input[2])
        m_output = xf.euler_matrix(e_output[0], e_output[1], e_output[2])
        m_left = m_xform @ m_input
        m_right = m_input @ m_xform
        if np.allclose(m_left, m_output):
            print('left \n{}\n * {} => {}'.format(m_to_str(m_xform), e_to_str(e_input), e_to_str(e_output)))
        if np.allclose(m_right, m_output):
            print('right {} * \n{}\n => {}'.format(e_to_str(e_input), m_to_str(m_xform), e_to_str(e_output)))
