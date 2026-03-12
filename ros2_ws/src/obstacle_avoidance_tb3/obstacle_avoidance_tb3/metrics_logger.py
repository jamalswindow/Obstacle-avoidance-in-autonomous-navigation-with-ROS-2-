#!/usr/bin/env python3
"""
Metrics Logger for Obstacle Avoidance Evaluation
Logs: min clearance, velocity, navigation time, recovery events,
      collisions, average velocities
Outputs CSV for report tables and graphs
"""

import time
import csv
import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        # Parameters
        self.declare_parameter('approach_name', 'unknown')
        self.approach_name = self.get_parameter(
            'approach_name').get_parameter_value().string_value

        # Output file
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.expanduser(
            '~/Desktop/AIS_Project_Obstacle_Avoidance/results')
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(
            self.output_dir,
            f'metrics_{self.approach_name}_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_sec', 'min_clearance_m', 'linear_vel', 'angular_vel',
            'pos_x', 'pos_y', 'distance_traveled_m', 'is_rotating',
            'collision'
        ])

        # State
        self.start_time = time.time()
        self.min_clearance_ever = float('inf')
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.rotation_count = 0
        self.was_rotating = False
        self.sample_count = 0
        self.clearance_sum = 0.0
        self.current_vel = Twist()
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

        # New metrics
        self.collision_count = 0
        self.recovery_count = 0
        self.was_backing_up = False
        self.linear_vel_sum = 0.0
        self.linear_vel_count = 0
        self.angular_vel_sum = 0.0
        self.angular_vel_count = 0
        self.current_min_clearance = float('inf')

        # Robot radius for collision detection
        self.robot_radius = 0.12

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Log every 0.5 seconds
        self.create_timer(0.5, self.log_metrics)

        # Print summary every 10 seconds
        self.create_timer(10.0, self.print_summary)

        self.get_logger().info(
            f'Metrics logging to: {self.csv_path} '
            f'(approach: {self.approach_name})')

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if 0.05 < r < msg.range_max]
        if valid:
            min_c = min(valid)
            self.current_min_clearance = min_c
            self.clearance_sum += min_c
            self.sample_count += 1
            if min_c < self.min_clearance_ever:
                self.min_clearance_ever = min_c
            # Collision detection: clearance less than robot radius
            if min_c < self.robot_radius:
                self.collision_count += 1
        else:
            self.current_min_clearance = float('inf')

    def vel_cb(self, msg):
        self.current_vel = msg

        # Track average linear velocity (only when moving)
        lin = msg.linear.x
        if abs(lin) > 0.01:
            self.linear_vel_sum += lin
            self.linear_vel_count += 1

        # Track average angular velocity
        ang = abs(msg.angular.z)
        if ang > 0.01:
            self.angular_vel_sum += ang
            self.angular_vel_count += 1

        # Recovery detection: backup behavior (negative linear velocity)
        is_backing_up = lin < -0.01
        if is_backing_up and not self.was_backing_up:
            self.recovery_count += 1
        self.was_backing_up = is_backing_up

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos_x = x
        self.current_pos_y = y

        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.total_distance += math.sqrt(dx*dx + dy*dy)

        self.last_x = x
        self.last_y = y

    def log_metrics(self):
        elapsed = time.time() - self.start_time
        lin_vel = self.current_vel.linear.x
        ang_vel = self.current_vel.angular.z
        clearance = self.current_min_clearance

        is_rotating = abs(ang_vel) > 0.5 and abs(lin_vel) < 0.05
        if is_rotating and not self.was_rotating:
            self.rotation_count += 1
        self.was_rotating = is_rotating

        collision = 1 if clearance < self.robot_radius else 0

        self.writer.writerow([
            f'{elapsed:.1f}',
            f'{clearance:.3f}',
            f'{lin_vel:.3f}',
            f'{ang_vel:.3f}',
            f'{self.current_pos_x:.3f}',
            f'{self.current_pos_y:.3f}',
            f'{self.total_distance:.3f}',
            int(is_rotating),
            collision
        ])
        self.csv_file.flush()

    def print_summary(self):
        elapsed = time.time() - self.start_time
        avg_clearance = (self.clearance_sum / self.sample_count
                         if self.sample_count > 0 else 0)
        avg_lin = (self.linear_vel_sum / self.linear_vel_count
                   if self.linear_vel_count > 0 else 0)
        avg_ang = (self.angular_vel_sum / self.angular_vel_count
                   if self.angular_vel_count > 0 else 0)
        self.get_logger().info(
            f'[{self.approach_name} {elapsed:.0f}s] '
            f'MinClear: {self.min_clearance_ever:.3f}m | '
            f'AvgClear: {avg_clearance:.3f}m | '
            f'Dist: {self.total_distance:.2f}m | '
            f'Rotations: {self.rotation_count} | '
            f'Collisions: {self.collision_count} | '
            f'Recoveries: {self.recovery_count} | '
            f'AvgLinVel: {avg_lin:.3f} | '
            f'AvgAngVel: {avg_ang:.3f}')

    def destroy_node(self):
        elapsed = time.time() - self.start_time
        avg_clearance = (self.clearance_sum / self.sample_count
                         if self.sample_count > 0 else 0)
        avg_lin = (self.linear_vel_sum / self.linear_vel_count
                   if self.linear_vel_count > 0 else 0)
        avg_ang = (self.angular_vel_sum / self.angular_vel_count
                   if self.angular_vel_count > 0 else 0)

        # Write summary file
        summary_path = self.csv_path.replace('.csv', '_summary.txt')
        with open(summary_path, 'w') as f:
            f.write(f'Approach: {self.approach_name}\n')
            f.write(f'Total Time: {elapsed:.1f} s\n')
            f.write(f'Distance Traveled: {self.total_distance:.2f} m\n')
            f.write(f'Min Clearance: {self.min_clearance_ever:.3f} m\n')
            f.write(f'Avg Clearance: {avg_clearance:.3f} m\n')
            f.write(f'Avg Linear Velocity: {avg_lin:.3f} m/s\n')
            f.write(f'Avg Angular Velocity: {avg_ang:.3f} rad/s\n')
            f.write(f'Collision Count: {self.collision_count}\n')
            f.write(f'Recovery Count: {self.recovery_count}\n')
            f.write(f'Rotation Events: {self.rotation_count}\n')
            f.write(f'Samples: {self.sample_count}\n')

        self.get_logger().info(f'Summary saved to: {summary_path}')
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
