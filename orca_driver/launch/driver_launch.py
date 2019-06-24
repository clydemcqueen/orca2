from launch import LaunchDescription
from launch_ros.actions import Node


# Launch just the driver, useful for seeing what the battery level is like


def generate_launch_description():
    return LaunchDescription([
        # Driver
        Node(package='orca_driver', node_executable='driver_node', output='screen',
             node_name='driver_node', parameters=[{
                'voltage_multiplier': 5.05,
                'thruster_4_reverse': True,  # Thruster 4 ESC is programmed incorrectly
                'tilt_channel': 6,
                'voltage_min': 14.0
            }]),
    ])
