from launch import LaunchDescription
from launch_ros.actions import Node


# Topside nodes, only useful if the AUV is out of the water and connected via wifi


def generate_launch_description():
    return LaunchDescription([
        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
            }]),




    ])
