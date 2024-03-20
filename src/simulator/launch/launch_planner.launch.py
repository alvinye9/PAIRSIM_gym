import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Run transform_path.py script node
    path_publisher = Node(
        package='simulator',
        executable='transform_path.py',
        output='screen',
        parameters=[
            {'csv_file': get_package_share_directory('simulator') + '/maps/raceline_traj_with_velocity_monza_edited.csv'},
            {'use_sim_time': False}
        ]
    )
    return LaunchDescription([
        path_publisher,
    ])
