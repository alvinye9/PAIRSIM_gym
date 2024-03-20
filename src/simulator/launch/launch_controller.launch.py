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


        # Node(
        #     package='long_control',
        #     executable='long_control',
        #     name='LongControlNode',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', logger],
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'mute': False},
        #         long_config
        #     ]
        # ),
        # Node(
        #     package='kin_control',
        #     executable='kin_control',
        #     name='KinControlNode',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', logger],
        #     #remappings=[('/joystick/steering_cmd', '/joystick/kin_control_steering_cmd')],
        #     parameters=[
        #         {'use_sim_time': False},
        #         {'mute': False},
        #         kin_config,
        #     ]
        # ),

        # Node(
        #     package='MPC',
        #     executable='MPC.py',
        #     name='MPC_Node',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', logger],
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #     ]
        # ),

        # Node(
        #     package='MPC',
        #     executable='long_mpc.py',
        #     name='Long_MPC_Node',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', logger],
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #     ]
        # ),

        # Node(
        #     package='accel_intf',
        #     executable='accel_intf',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', logger],
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         accel_config
        #     ]
        # ),

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

