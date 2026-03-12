#!/bin/bash
# Sends sequential navigation goals in a loop for continuous robot movement
# The robot will navigate between waypoints in the TurtleBot3 world

source /opt/ros/humble/setup.bash

echo "Setting initial pose at spawn position (-2.0, -0.5)..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -2.0, "y": -0.5, "z": 0.0}, "orientation": {"w": 1.0}}}}'

sleep 10
echo "Starting continuous navigation loop..."

# Waypoints around the TurtleBot3 world map
GOALS=(
  "0.5 0.5 0.0 0.0 0.0 0.0 1.0"
  "0.5 -0.5 0.0 0.0 0.0 0.7 0.7"
  "-0.5 -0.5 0.0 0.0 0.0 1.0 0.0"
  "-0.5 0.5 0.0 0.0 0.0 -0.7 0.7"
  "0.0 0.0 0.0 0.0 0.0 0.0 1.0"
)

ROUND=1
while true; do
  echo ""
  echo "===== Round $ROUND ====="
  for i in "${!GOALS[@]}"; do
    read -r px py pz ox oy oz ow <<< "${GOALS[$i]}"
    echo "Navigating to waypoint $((i+1)): ($px, $py)..."

    RESULT=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{\"pose\": {\"header\": {\"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": $px, \"y\": $py, \"z\": $pz}, \"orientation\": {\"x\": $ox, \"y\": $oy, \"z\": $oz, \"w\": $ow}}}}" 2>&1)

    echo "$RESULT" | tail -3

    if echo "$RESULT" | grep -q "SUCCEEDED"; then
      echo "Reached waypoint $((i+1))!"
    elif echo "$RESULT" | grep -q "rejected"; then
      echo "Goal rejected, waiting 5 seconds and retrying..."
      sleep 5
    else
      echo "Goal did not succeed, moving to next waypoint..."
    fi

    sleep 2
  done
  ROUND=$((ROUND + 1))
done
