import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='obstacle_avoidance_tb3',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='turtlebot3_node',
            executable='turtlebot3_laser_scan',
            name='laser_scan',
            output='screen'
        )
    ])

