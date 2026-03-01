#!/usr/bin/env python3

import math
import time
import csv
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus


class NavigationMetrics(Node):
    def __init__(self):
        super().__init__('navigation_metrics')

        # Parameters
        self.declare_parameter('planner_name', 'dwb')
        self.declare_parameter('output_file', os.path.expanduser(
            '~/ros2_ws_new_file/ros2_ws/results/metrics.csv'))

        self.planner_name = self.get_parameter('planner_name').value
        self.output_file = self.get_parameter('output_file').value

        # Predefined goal poses for TurtleBot3 world (x, y, yaw)
        # Robot starts at (-2.0, -0.5, 0.0)
        self.goal_poses = [
            (0.5, 0.0, 0.0),       # Center-right area
            (-0.5, -1.0, 1.57),    # Center-left bottom
            (0.5, 1.0, 3.14),      # Right-top area
            (-1.0, 0.5, -1.57),    # Left-top area
            (-2.0, -0.5, 0.0),     # Back to start
        ]

        self.current_goal_idx = 0
        self.reset_metrics()

        # State flags
        self.navigating = False
        self.finished = False
        self.waiting_for_next = False

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Results storage
        self.all_results = []

        self.get_logger().info(f'Navigation Metrics node started (planner: {self.planner_name})')
        self.get_logger().info('Waiting for Nav2 action server...')

        # Main loop timer - drives all state transitions
        self.main_timer = self.create_timer(2.0, self.main_loop)
        self.wait_start = None

    def reset_metrics(self):
        self.start_time = None
        self.elapsed_time = 0.0
        self.path_length = 0.0
        self.min_obstacle_clearance = float('inf')
        self.collision_count = 0
        self.vel_samples = 0
        self.total_linear_vel = 0.0
        self.total_angular_vel = 0.0
        self.heading_changes = 0.0
        self.last_heading = None
        self.last_pos = None

    def main_loop(self):
        if self.finished:
            return

        if self.navigating or self.waiting_for_next:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().info('Still waiting for Nav2...')
            return

        if self.current_goal_idx >= len(self.goal_poses):
            self.finish_all_tests()
            return

        # Add delay between goals
        if self.wait_start is not None:
            if time.time() - self.wait_start < 5.0:
                return
            self.wait_start = None

        self.send_goal()

    def send_goal(self):
        x, y, yaw = self.goal_poses[self.current_goal_idx]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.reset_metrics()
        self.start_time = time.time()
        self.navigating = True

        self.get_logger().info(
            f'=== Goal {self.current_goal_idx + 1}/{len(self.goal_poses)}: '
            f'({x:.2f}, {y:.2f}, yaw={yaw:.2f}) ===')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            self.navigating = False
            self.current_goal_idx += 1
            self.wait_start = time.time()
            return

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # TODO: implement result processing and metric recording
        pass

    def finish_all_tests(self):
        # TODO: implement summary and CSV output
        pass

    def save_results(self):
        # TODO: implement CSV writing
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMetrics()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
