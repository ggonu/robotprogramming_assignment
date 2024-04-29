from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment2',
            executable='node1',
            name='node1'
        ),
        Node(
            package='assignment2',
            executable='node2',
            name='node2',
            arguments=['10', '10']
        )
    ])
    