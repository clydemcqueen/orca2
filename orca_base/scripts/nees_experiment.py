#!/usr/bin/env python3

"""
Run NEES experiment(s)

Usage with filter:
ros2 run orca_base nees_experiment.py

Usage without filter:
ros2 run orca_base nees_experiment.py --ros-args -r filtered_fp:=/forward_camera/fp
"""

import nees_fp
import numpy as np
import rclpy
import rclpy.logging
from mission_experiment import MissionExperiment, MissionExperimentRunNode
from orca_util import seconds


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
random_markers_experiment = MissionExperiment.go_to_markers('random markers', 10, [], [], [], True, calculate_nees)


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
