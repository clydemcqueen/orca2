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
Run NEES experiment(s).

Usage with filter:
ros2 run orca_base nees_experiment.py

Usage without filter:
ros2 run orca_base nees_experiment.py --ros-args -r filtered_fp:=/forward_camera/fp
"""

from mission_experiment import MissionExperiment, MissionExperimentRunNode
import nees_fp
import numpy as np
from orca_util import seconds
import rclpy
import rclpy.logging


# Calc and report NEES
# Note: this takes several seconds -- way too long for a callback
# The sub is drifting while we're waiting, which often means that subsequent experiments fail
# TODO run on a different thread
def calculate_nees(ex: MissionExperiment):
    if len(ex.fp_msgs) < 5:
        print('too few fp messages')
        return

    print('{} fp messages spanning {} seconds'.format(
        len(ex.fp_msgs),
        seconds(ex.fp_msgs[-1].header.stamp) - seconds(ex.fp_msgs[0].header.stamp)))

    if len(ex.gt_msgs) < 5:
        print('too few ground truth messages')
        return

    print('{} ground truth messages spanning {} seconds'.format(
        len(ex.gt_msgs),
        seconds(ex.gt_msgs[-1].header.stamp) - seconds(ex.gt_msgs[0].header.stamp)))

    nees_values = nees_fp.nees(ex.fp_msgs, ex.gt_msgs)

    if nees_values:
        print('average NEES value {}'.format(np.mean(nees_values)))
    else:
        print('no NEES values')


# Run through all markers in random order, 10 times
random_markers_experiment = MissionExperiment.go_to_markers('random markers', 10, [], [], [], True,
                                                            False, calculate_nees)


def main(args=None):
    rclpy.init(args=args)

    node = MissionExperimentRunNode(experiments=[random_markers_experiment])

    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.stop_mission_and_destroy_client()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
