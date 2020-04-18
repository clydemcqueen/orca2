from launch import LaunchDescription
from launch_ros.actions import Node


# Test the hardware


def generate_launch_description():
    return LaunchDescription([
        Node(package='orca_driver', node_executable='barometer_node', output='screen', node_name='barometer_node'),

        Node(package='orca_driver', node_executable='driver_node', output='screen', node_name='driver_node',
             parameters=[{
                 'thruster_4_reverse': True  # Thruster 4 ESC is programmed incorrectly TODO ?
             }]),

        Node(package='orca_driver', node_executable='test_node', output='screen'),
    ])
