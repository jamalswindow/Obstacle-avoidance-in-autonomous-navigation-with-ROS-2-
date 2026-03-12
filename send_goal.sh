#!/bin/bash
# Check if Gazebo is running
echo "Checking if Gazebo is running..."
if ! ros2 topic list 2>/dev/null | grep -q "/odom"; then
    echo "ERROR: /odom topic not found. Is Gazebo running?"
    echo "Start Gazebo first: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
    exit 1
fi

# Set initial pose at TurtleBot3 spawn position (-2.0, -0.5)
echo "Setting initial pose at spawn position (-2.0, -0.5)..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -2.0, "y": -0.5, "z": 0.0}, "orientation": {"w": 1.0}}}}'

echo "Waiting 10 seconds for localization..."
sleep 10

# Send navigation goal toward center of the map
echo "Sending navigation goal to (0.5, 0.5)..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{"pose": {"header": {"frame_id": "map"}, "pose": {"position": {"x": 0.5, "y": 0.5, "z": 0.0}, "orientation": {"w": 1.0}}}}'
