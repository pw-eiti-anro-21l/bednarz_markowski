from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
            ),
        Node(
            package='lab1',
            executable='my_teleop',
            name='program',
            prefix='gnome-terminal --',
            parameters = [{'up':'w', 'left':'a', 'right':'d', 'down':'s'}]
            )
        ])