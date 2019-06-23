from launch import LaunchDescription
from launch_ros.actions import Node


# Launch pt2 driver and hardware test


def generate_launch_description():
    return LaunchDescription([
        # Driver node handles thrusters, lights and camera tilt
        Node(package='orca_driver', node_executable='driver_node', output='screen',
             node_name='driver_node', parameters=[{
                'voltage_multiplier': 5.05,
                'thruster_4_reverse': True,  # Thruster 4 ESC is programmed incorrectly
                'tilt_channel': 6,
                'voltage_min': 0.0  # 0.0 For bench testing TODO
            }]),

        # Tester
        Node(package='orca_driver', node_executable='test_node', output='screen'),
    ])
