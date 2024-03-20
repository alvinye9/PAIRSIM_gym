import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_descr = []
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("simulator"), "launch", "launch_planner.launch.py")
        )
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("simulator"), "launch", "launch_controller.launch.py")
        )
    )

    launch_descr.extend([
        planner,
        controller
    ])

    return LaunchDescription(launch_descr)