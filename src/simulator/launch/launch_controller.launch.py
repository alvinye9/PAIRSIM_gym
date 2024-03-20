import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = False
    logger = launch.substitutions.LaunchConfiguration("log_level")

    kin_package = get_package_share_directory('kin_control')
    kin_config = kin_package + '/config/params.yaml'

    long_package = get_package_share_directory('long_control')
    long_config = long_package + '/config/params.yaml'

    accel_intf_package = get_package_share_directory('accel_intf')
    accel_config = accel_intf_package + '/config/params.yaml'

    # odom_int_package = get_package_share_directory('wheel_odometer_integrator')
    # odom_launch = '/launch/odom_fusion.launch.py'

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),


        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            arguments=['--ros-args', '--log-level', logger],
            parameters=[
                {'use_sim_time': use_sim_time},
            ]
        ),


    ])

