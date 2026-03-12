import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('obstacle_avoidance_tb3')

    mppi_config = os.path.join(pkg_share, 'config', 'mppi_config.yaml')
    map_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': mppi_config,
        }.items(),
    )

    # Launch waypoint sender after a delay so Nav2 has time to initialize
    waypoint_sender = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='obstacle_avoidance_tb3',
                executable='waypoint_sender',
                name='waypoint_sender',
                output='screen',
                parameters=[{'use_sim_time': False}],
            )
        ],
    )

    metrics_logger = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='obstacle_avoidance_tb3',
                executable='metrics_logger',
                name='metrics_logger',
                output='screen',
                parameters=[{'use_sim_time': False, 'approach_name': 'mppi'}],
            )
        ],
    )

    return LaunchDescription([
        bringup_launch,
        waypoint_sender,
        metrics_logger,
    ])
