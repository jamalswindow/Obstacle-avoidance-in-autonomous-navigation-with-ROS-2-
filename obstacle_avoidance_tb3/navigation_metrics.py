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

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

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
        if not self.navigating:
            return  # Prevent duplicate callbacks

        result = future.result()
        status = result.status
        self.navigating = False
        self.elapsed_time = time.time() - self.start_time

        if status == GoalStatus.STATUS_SUCCEEDED:
            status_str = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_ABORTED:
            status_str = 'ABORTED'
        elif status == GoalStatus.STATUS_CANCELED:
            status_str = 'CANCELED'
        else:
            status_str = f'UNKNOWN({status})'

        # Compute averages
        avg_linear = self.total_linear_vel / max(1, self.vel_samples)
        avg_angular = self.total_angular_vel / max(1, self.vel_samples)

        goal = self.goal_poses[self.current_goal_idx]
        result_data = {
            'planner': self.planner_name,
            'goal_idx': self.current_goal_idx + 1,
            'goal_x': goal[0],
            'goal_y': goal[1],
            'goal_yaw': goal[2],
            'status': status_str,
            'time_sec': round(self.elapsed_time, 2),
            'path_length_m': round(self.path_length, 3),
            'min_clearance_m': round(self.min_obstacle_clearance, 3) if self.min_obstacle_clearance != float('inf') else -1,
            'collisions': self.collision_count,
            'avg_linear_vel': round(avg_linear, 4),
            'avg_angular_vel': round(avg_angular, 4),
            'heading_changes_rad': round(self.heading_changes, 3),
        }
        self.all_results.append(result_data)

        self.get_logger().info(f'--- Goal {self.current_goal_idx + 1} Result ---')
        self.get_logger().info(f'  Status:           {status_str}')
        self.get_logger().info(f'  Time:             {self.elapsed_time:.2f} s')
        self.get_logger().info(f'  Path Length:      {self.path_length:.3f} m')
        self.get_logger().info(f'  Min Clearance:    {result_data["min_clearance_m"]} m')
        self.get_logger().info(f'  Collisions:       {self.collision_count}')
        self.get_logger().info(f'  Avg Linear Vel:   {avg_linear:.4f} m/s')
        self.get_logger().info(f'  Avg Angular Vel:  {avg_angular:.4f} rad/s')
        self.get_logger().info(f'  Heading Changes:  {self.heading_changes:.3f} rad')

        self.current_goal_idx += 1
        self.wait_start = time.time()

    def odom_callback(self, msg):
        if not self.navigating:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        heading = 2.0 * math.atan2(qz, qw)

        if self.last_pos is not None:
            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            self.path_length += math.sqrt(dx * dx + dy * dy)

        if self.last_heading is not None:
            dh = abs(heading - self.last_heading)
            if dh > math.pi:
                dh = 2.0 * math.pi - dh
            self.heading_changes += dh

        self.last_pos = (x, y)
        self.last_heading = heading

    def scan_callback(self, msg):
        if not self.navigating:
            return

        valid_ranges = [r for r in msg.ranges
                        if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_range = min(valid_ranges)
            if min_range < self.min_obstacle_clearance:
                self.min_obstacle_clearance = min_range
            if min_range < 0.105:
                self.collision_count += 1

    def cmd_vel_callback(self, msg):
        if not self.navigating:
            return
        self.total_linear_vel += abs(msg.linear.x)
        self.total_angular_vel += abs(msg.angular.z)
        self.vel_samples += 1

    def finish_all_tests(self):
        if self.finished:
            return
        self.finished = True
        self.main_timer.cancel()

        self.get_logger().info('========================================')
        self.get_logger().info(f'All {len(self.goal_poses)} goals completed!')
        self.get_logger().info(f'Planner: {self.planner_name}')
        self.get_logger().info('========================================')

        succeeded = [r for r in self.all_results if r['status'] == 'SUCCEEDED']
        if succeeded:
            avg_time = sum(r['time_sec'] for r in succeeded) / len(succeeded)
            avg_path = sum(r['path_length_m'] for r in succeeded) / len(succeeded)
            clearances = [r['min_clearance_m'] for r in succeeded if r['min_clearance_m'] > 0]
            avg_clearance = sum(clearances) / max(1, len(clearances))
            total_collisions = sum(r['collisions'] for r in succeeded)

            self.get_logger().info(f'  Succeeded: {len(succeeded)}/{len(self.all_results)}')
            self.get_logger().info(f'  Avg Time:       {avg_time:.2f} s')
            self.get_logger().info(f'  Avg Path:       {avg_path:.3f} m')
            self.get_logger().info(f'  Avg Clearance:  {avg_clearance:.3f} m')
            self.get_logger().info(f'  Total Collisions: {total_collisions}')
        else:
            self.get_logger().warn('No goals succeeded!')

        self.save_results()

        # Shutdown after saving
        self.get_logger().info('Test complete. Shutting down...')
        raise SystemExit(0)

    def save_results(self):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        file_exists = os.path.isfile(self.output_file)
        fieldnames = [
            'planner', 'goal_idx', 'goal_x', 'goal_y', 'goal_yaw',
            'status', 'time_sec', 'path_length_m', 'min_clearance_m',
            'collisions', 'avg_linear_vel', 'avg_angular_vel', 'heading_changes_rad'
        ]

        with open(self.output_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            for row in self.all_results:
                writer.writerow(row)

        self.get_logger().info(f'Results saved to {self.output_file}')


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
