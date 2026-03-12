# Obstacle Avoidance in Autonomous Navigation with ROS 2

**Project Group:** OBS-1
**Student:** Md Jamal Uddin (1387201)
**Course:** Autonomous Intelligent Systems, Frankfurt University of Applied Sciences
**Supervisor:** Prof. Dr. Peter Nauth

## Overview
Comparative evaluation of obstacle avoidance strategies for autonomous mobile robots using the TurtleBot3 platform in ROS 2. This project implements and compares four approaches:

1. **Reactive** — Custom 5-zone LiDAR-based obstacle avoidance (no Nav2)
2. **DWB** — Dynamic Window Approach B (Nav2 default controller)
3. **TEB** — Timed Elastic Band local planner (Nav2 plugin)
4. **MPPI** — Model Predictive Path Integral controller (Nav2 plugin)

## Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot3 packages (`ros-humble-turtlebot3*`)
- Nav2 stack (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- Gazebo simulator

## Setup
```bash
cd ~/Desktop/AIS_Project_Obstacle_Avoidance/ros2_ws
colcon build --packages-select obstacle_avoidance_tb3
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Running Tests

### 1. Start Gazebo (in Terminal 1)
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run a planner (in Terminal 2)

**Reactive approach:**
```bash
ros2 launch obstacle_avoidance_tb3 laser_scan.launch.py
```

**DWB approach:**
```bash
ros2 launch obstacle_avoidance_tb3 dwb_navigation.launch.py
```

**TEB approach:**
```bash
ros2 launch obstacle_avoidance_tb3 teb_navigation.launch.py
```

**MPPI approach:**
```bash
ros2 launch obstacle_avoidance_tb3 mppi_navigation.launch.py
```

### 3. Metrics
Results are saved to `results/` directory as CSV files with approach name and timestamp.

## Project Structure
```
ros2_ws/src/obstacle_avoidance_tb3/
├── config/
│   ├── dwa_config.yaml          # DWB controller parameters
│   ├── teb_config.yaml          # TEB controller parameters
│   └── mppi_config.yaml         # MPPI controller parameters
├── launch/
│   ├── laser_scan.launch.py     # Reactive approach
│   ├── dwb_navigation.launch.py # DWB with Nav2
│   ├── teb_navigation.launch.py # TEB with Nav2
│   └── mppi_navigation.launch.py# MPPI with Nav2
├── obstacle_avoidance_tb3/
│   ├── obstacle_avoidance.py    # Reactive obstacle avoidance node
│   ├── metrics_logger.py        # Performance metrics collection
│   └── waypoint_sender.py       # Automatic Nav2 goal sender
├── package.xml
├── setup.py
└── setup.cfg
```
