from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='repub',
            executable='repub_ct_node.py',
            name = 'repub_ct_node'
        )
    ])
