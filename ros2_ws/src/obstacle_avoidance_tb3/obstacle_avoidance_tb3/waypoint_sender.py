#!/usr/bin/env python3
"""
Automatic Waypoint Sender for Nav2
Sends a loop of navigation goals so DWB/TEB can be tested without RViz.
Publishes initial pose programmatically to bypass AMCL race condition.

IMPORTANT: This node runs with use_sim_time=False (set in launch file)
so that timers use wall clock. VirtualBox runs Gazebo at ~0.1-0.2 RTF,
making sim-time timers 5-10x slower than expected.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
import math


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Publisher for initial pose
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Lifecycle state client to check bt_navigator readiness
        self._lifecycle_client = self.create_client(
            GetState, '/bt_navigator/get_state')

        # Waypoints around TurtleBot3 World (x, y, yaw_deg)
        self.waypoints = [
            ( 1.5,  0.0,   0),
            ( 2.0,  1.0,  90),
            ( 1.0,  1.5, 180),
            ( 0.0,  1.5, 180),
            (-1.0,  1.0, 225),
            (-1.5,  0.0, 270),
            (-1.0, -1.0, 315),
            ( 0.0, -1.5,   0),
            ( 1.0, -1.0,  45),
            ( 0.5,  0.0,  90),
        ]

        self.current_wp = 0
        self._started = False
        self._goal_active = False

        self.get_logger().info(
            f'Waypoint sender ready with {len(self.waypoints)} waypoints.')

        # Publish initial pose every 5 wall seconds until Nav2 is active
        self._timer = self.create_timer(5.0, self._tick)
        self._phase = 'wait_for_nav2'

    def _publish_initial_pose(self):
        """Publish initial pose to /initialpose topic.
        Use stamp=0 so AMCL uses latest available transform
        (avoids wall-clock vs sim-time mismatch in VirtualBox)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)

    def _tick(self):
        """Main polling loop on wall timer — keeps trying until Nav2 is ready."""
        # Always publish initial pose while waiting
        self._publish_initial_pose()

        if self._phase == 'wait_for_nav2':
            # Check if bt_navigator lifecycle service exists
            if not self._lifecycle_client.service_is_ready():
                self.get_logger().info(
                    'Waiting for Nav2 (bt_navigator not yet available)...')
                return

            # Check lifecycle state
            req = GetState.Request()
            future = self._lifecycle_client.call_async(req)
            future.add_done_callback(self._on_lifecycle_response)

        elif self._phase == 'ready' and not self._started:
            self._started = True
            self._timer.cancel()
            self.get_logger().info('Nav2 is ACTIVE! Sending first goal...')
            self.send_next_goal()

    def _on_lifecycle_response(self, future):
        try:
            result = future.result()
            state = result.current_state.label
            self.get_logger().info(f'bt_navigator state: {state}')
            if state == 'active':
                self._phase = 'ready'
            else:
                self.get_logger().info(
                    f'Nav2 not ready yet (state={state}), will retry...')
        except Exception as e:
            self.get_logger().warn(f'Failed to get lifecycle state: {e}')

    def send_next_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Action server not available. Retrying in 5s...')
            self.create_timer(5.0, self.send_next_goal)
            return

        x, y, yaw_deg = self.waypoints[self.current_wp]
        yaw = math.radians(yaw_deg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = Time(sec=0, nanosec=0)
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[WP {self.current_wp + 1}/{len(self.waypoints)}] '
            f'Sending goal: ({x:.1f}, {y:.1f}, {yaw_deg}deg)')

        self._goal_active = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Retrying in 5s...')
            self._goal_active = False
            self.create_timer(5.0, self.send_next_goal)
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        pass

    def result_cb(self, future):
        self._goal_active = False
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted. Moving to next.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal canceled. Moving to next.')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')

        self.advance_and_send()

    def advance_and_send(self):
        self.current_wp = (self.current_wp + 1) % len(self.waypoints)
        if self.current_wp == 0:
            self.get_logger().info('=== Completed full loop! Starting again. ===')
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
