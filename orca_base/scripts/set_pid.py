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
Set pid coefficients.

The coefficients must be parameters of the form prefix_kp, prefix_ki and prefix_kd.

Usage:
ros2 run orca_base set_pid.py node prefix Ku Tu
ros2 run orca_base set_pid.py /rov_node rov_pressure_pid_ 0.001 3
"""

import subprocess
import sys


# Ziegler Nichols PID coefficients
# See https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

def zn_classic_pid(Ku, Tu):
    return [0.6 * Ku, 1.2 * Ku / Tu, 0.075 * Ku * Tu]


def zn_pessen(Ku, Tu):
    return [0.7 * Ku, 1.75 * Ku / Tu, 0.105 * Ku * Tu]


def zn_no_overshoot(Ku, Tu):
    return [0.2 * Ku, 0.4 * Ku / Tu, 0.0667 * Ku * Tu]


# If we have a decent Kp and know Tu (oscillation period), we can find a corresponding Kd:
#       Classic:        Kd = 0.125 * Kp * Tu
#       Pessen:         Kd = 0.15 * Kp * Tu
#       No overshoot:   Kd = 0.333 * Kp * Tu
#
# And a Ki:
#       Classic:        Kd = 2.0 * Kp / Tu
#       Pessen:         Kd = 2.5 * Kp / Tu
#       No overshoot:   Kd = 2.0 * Kp / Tu

def zn_classic_from_kp(Kp, Tu):
    return [Kp, 2.0 * Kp / Tu, 0.125 * Kp * Tu]


def zn_pessen_from_kp(Kp, Tu):
    return [Kp, 2.5 * Kp / Tu, 0.15 * Kp * Tu]


def zn_no_overshoot_from_kp(Kp, Tu):
    return [Kp, 2.0 * Kp / Tu, 0.333 * Kp * Tu]


def main(args):
    print(args)
    if len(args) != 5:
        print('Usage: python3 set_pid.py node prefix Ku Tu')
        return

    node = args[1]
    prefix = args[2]
    Ku = float(args[3])
    Tu = float(args[4])
    Kp, Ki, Kd = zn_classic_pid(Ku, Tu)

    Kp_str = format(Kp, '.10f')
    Ki_str = format(Ki, '.10f')
    Kd_str = format(Kd, '.10f')

    print('Set {}[{}, {}, {}] on {}'.format(prefix, Kp_str, Ki_str, Kd_str, node))

    subprocess.run(['ros2', 'param', 'set', node, prefix + 'kp', Kp_str])
    subprocess.run(['ros2', 'param', 'set', node, prefix + 'ki', Ki_str])
    subprocess.run(['ros2', 'param', 'set', node, prefix + 'kd', Kd_str])


if __name__ == '__main__':
    main(sys.argv)
