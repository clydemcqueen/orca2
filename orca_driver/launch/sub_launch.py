from launch import LaunchDescription
from launch_ros.actions import Node


# Launch sub nodes


def generate_launch_description():
    return LaunchDescription([
        # Driver
        Node(package='orca_driver', node_executable='driver_node', output='log',
             node_name='driver_node', parameters=[{
                'voltage_multiplier': 5.05,
                'thruster_4_reverse': True,  # Thruster 4 ESC is programmed incorrectly TODO ?
                'tilt_channel': 6,
                'voltage_min': 12.0
            }]),
    ])