import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    republisher = Node(
        package='repub',
        executable='repub_node',
        output='screen',
    )
    return LaunchDescription([
        republisher,
    ])
