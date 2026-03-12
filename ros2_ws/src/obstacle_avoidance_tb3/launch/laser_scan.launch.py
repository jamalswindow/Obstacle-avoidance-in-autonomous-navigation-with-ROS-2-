from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_avoidance_tb3',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='obstacle_avoidance_tb3',
            executable='metrics_logger',
            name='metrics_logger',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'approach_name': 'reactive'},
            ],
        ),
    ])
