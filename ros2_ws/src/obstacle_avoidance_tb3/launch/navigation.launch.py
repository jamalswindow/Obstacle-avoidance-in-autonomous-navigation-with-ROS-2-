from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'planner_server.local_planner': 'dwb_core::DWAPlanner'},
                '/home/maruf/ros2_ws/src/obstacle_avoidance_tb3/config/dwa_config.yaml'
            ]
        ),
    ])
