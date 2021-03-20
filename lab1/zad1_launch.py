from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
            ),
        Node(
            package='lab1',
            namespace='lab1',
            executable='parameters',
            name='program'
            )
        ])