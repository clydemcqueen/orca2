#!/usr/bin/env python

"""
Given a set of input and output orientations, find all "right-angle" transforms that satisfy
    xform @ input => output  --or--
    input @ xform => output
"""

import math
import numpy as np
import transformations as xf


# Pretty-print Euler [r, p, y]
def e_to_str(e):
    return '[r={: 06.5f}, p={: 06.5f}, y={: 06.5f}]'.format(e[0], e[1], e[2])


# Pretty-print Quaternion [w, x, y, z]
def q_to_str(q):
    return '[w={: 06.5f}, x={: 06.5f}, y={: 06.5f}, z={: 06.5f}]'.format(q[0], q[1], q[2], q[3])


# Generate all possible Euler rotations, in pi/2 increments
def gen_all_euler_rotations():
    for roll in [0, math.pi / 2, math.pi, -math.pi / 2]:
        for pitch in [0, math.pi / 2, math.pi, -math.pi / 2]:
            for yaw in [0, math.pi / 2, math.pi, -math.pi / 2]:
                yield roll, pitch, yaw


# Generate all unique right-angle Quaternion rotations
def gen_unique_q_rotations():
    unique = []
    for r in list(gen_all_euler_rotations()):
        q = xf.quaternion_from_euler(r[0], r[1], r[2])
        hit = False
        for candidate in unique:
            if np.allclose(q, candidate[1]):
                # print('{} gives same q as {}'.format(e_to_str(r), e_to_str(candidate[0])))
                hit = True
                break
        if not hit:
            # print('adding', q_to_str(q))
            unique.append([r, q])

    result = []
    for final in unique:
        result.append(final[1])
    return result


unique_q_rotations = gen_unique_q_rotations()
print('{} unique quaternions'.format(len(unique_q_rotations)))

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
for q_xform in unique_q_rotations:
    m_xform = xf.quaternion_matrix(q_xform)
    for e_input, e_output in zip(e_inputs, e_outputs):
        m_input = xf.euler_matrix(e_input[0], e_input[1], e_input[2])
        m_output = xf.euler_matrix(e_output[0], e_output[1], e_output[2])
        m_left = m_xform @ m_input  # np.dot(m_xform, m_input)
        m_right = m_input @ m_xform  # np.dot(m_input, m_xform)
        if np.allclose(m_left, m_output):
            print('left {} * {} => {}'.format(q_to_str(q_xform), e_to_str(e_input), e_to_str(e_output)))
        if np.allclose(m_right, m_output):
            print('right {} * {} => {}'.format(e_to_str(e_input), q_to_str(q_xform), e_to_str(e_output)))
