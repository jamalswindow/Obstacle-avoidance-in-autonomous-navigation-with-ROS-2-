import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    planner = LaunchConfiguration('planner').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('obstacle_avoidance_tb3')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # Select params file based on planner argument
    if planner == 'teb':
        params_file = os.path.join(pkg_share, 'config', 'nav2_teb_params.yaml')
    else:
        params_file = os.path.join(pkg_share, 'config', 'nav2_dwb_params.yaml')

    map_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    rviz_config = os.path.join(tb3_nav2_dir, 'rviz', 'tb3_navigation2.rviz')

    # Launch Gazebo with TurtleBot3 world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Launch Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
        }.items()
    )

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return [gazebo_launch, nav2_launch, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        DeclareLaunchArgument(
            'planner',
            default_value='dwb',
            description='Local planner to use: dwb or teb'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock'
        ),

        OpaqueFunction(function=launch_setup),
    ])
