#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Create publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber to read LIDAR data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        """Process LIDAR data and avoid obstacles"""
        ranges = msg.ranges
        front_range = min(min(ranges[:30] + ranges[-30:]), 10.0)  # Check front area

        twist = Twist()
        if front_range < 0.5:  # If an obstacle is closer than 0.5m
            self.get_logger().info("Obstacle detected! Turning left...")
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn left
        else:
            self.get_logger().info("Moving forward...")
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
