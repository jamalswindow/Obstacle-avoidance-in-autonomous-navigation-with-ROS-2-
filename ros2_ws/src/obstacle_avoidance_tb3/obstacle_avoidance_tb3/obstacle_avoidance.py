#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    # Speed limits
    MAX_LINEAR_SPEED = 0.22   # TurtleBot3 Burger max: 0.22 m/s
    MAX_ANGULAR_SPEED = 2.84  # TurtleBot3 Burger max: 2.84 rad/s

    # Distance thresholds
    STOP_DISTANCE = 0.25      # Emergency stop distance (m)
    SLOW_DISTANCE = 0.50      # Start slowing down (m)
    SAFE_DISTANCE = 1.0       # Full speed ahead (m)

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Anti-oscillation state
        self._last_obstacle_time = self.get_clock().now()
        self._turn_cooldown = 2.0  # seconds to keep turning after obstacle detected

    def get_zone_min(self, ranges, start_deg, end_deg, range_max):
        """Get minimum valid reading in a degree zone (0° = front, CCW)."""
        valid = []
        for deg in range(start_deg, end_deg):
            idx = deg % len(ranges)
            r = ranges[idx]
            if 0.1 < r < range_max:
                valid.append(r)
        return min(valid) if valid else range_max

    def scan_callback(self, msg):
        """Process LIDAR data using 5-zone sensing with anti-oscillation."""
        ranges = msg.ranges
        if len(ranges) == 0:
            return

        # 5 zones (degrees, 0° = front, CCW positive)
        front       = self.get_zone_min(ranges, 340, 380, msg.range_max)
        front_left  = self.get_zone_min(ranges, 20, 70, msg.range_max)
        front_right = self.get_zone_min(ranges, 290, 340, msg.range_max)
        left        = self.get_zone_min(ranges, 70, 110, msg.range_max)
        right       = self.get_zone_min(ranges, 250, 290, msg.range_max)

        twist = Twist()
        now = self.get_clock().now()

        # Check if we're still in cooldown from a recent turn maneuver
        in_cooldown = (now - self._last_obstacle_time).nanoseconds / 1e9 < self._turn_cooldown

        # --- Determine if obstacle requires action ---
        obstacle_ahead = front < self.SAFE_DISTANCE

        if obstacle_ahead:
            self._last_obstacle_time = now

        # --- Determine linear speed ---
        if front < self.STOP_DISTANCE:
            linear_speed = 0.0
        elif front < self.SAFE_DISTANCE or in_cooldown:
            ratio = (front - self.STOP_DISTANCE) / (self.SAFE_DISTANCE - self.STOP_DISTANCE)
            ratio = max(0.0, min(1.0, ratio))
            # During cooldown, move slower to complete the avoidance maneuver
            if in_cooldown:
                linear_speed = ratio * self.MAX_LINEAR_SPEED * 0.4
            else:
                linear_speed = ratio * self.MAX_LINEAR_SPEED
        else:
            if in_cooldown:
                linear_speed = self.MAX_LINEAR_SPEED * 0.6
            else:
                linear_speed = self.MAX_LINEAR_SPEED

        # --- Determine angular speed ---
        angular_speed = 0.0

        if obstacle_ahead or in_cooldown:
            right_weight = (1.0 / max(front_right, 0.1)) + (0.5 / max(right, 0.1))
            left_weight  = (1.0 / max(front_left, 0.1))  + (0.5 / max(left, 0.1))

            angular_speed = (right_weight - left_weight) * 0.5
            angular_speed = max(-self.MAX_ANGULAR_SPEED, min(angular_speed, self.MAX_ANGULAR_SPEED))

            if front < self.SLOW_DISTANCE and abs(angular_speed) < 0.3:
                if front_left >= front_right:
                    angular_speed = 1.0
                else:
                    angular_speed = -1.0

            if front < self.STOP_DISTANCE:
                linear_speed = 0.0
                angular_speed = 1.5 if left >= right else -1.5

        # Gentle wall-following when path is clear but sides are close
        elif front_left < 0.4:
            angular_speed = -0.3
        elif front_right < 0.4:
            angular_speed = 0.3

        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f"F:{front:.2f} FL:{front_left:.2f} FR:{front_right:.2f} "
            f"L:{left:.2f} R:{right:.2f} | "
            f"v:{linear_speed:.2f} w:{angular_speed:.2f}"
            f"{' [COOLDOWN]' if in_cooldown else ''}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

