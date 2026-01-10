## Introduction
Obstacle avoidance is a critical aspect of autonomous navigation systems. This project leverages ROS 2 (Robot Operating System 2) and the TurtleBot3 platform to implement and enhance an obstacle avoidance algorithm using the Navigation 2 (Nav2) stack. The system is designed to detect and avoid obstacles in real-time while ensuring efficient path planning and collision-free movement. **Navigation 2 (Nav2) stack**.

## Objective
The primary goal is to enable the TurtleBot3 to navigate autonomously in a dynamic environment while avoiding obstacles in real time. This project is part of the Autonomous Systems and Robotics course at Frankfurt University of Applied Sciences.
**Autonomous Systems and Robotics course at Frankfurt University of Applied Sciences**.

## System Architecture

### Hardware Requirements
- TurtleBot3 (Burger or Waffle Pi)
- Lidar Sensor
- Ubuntu 22.04-based computer

### Software Requirements
- ROS 2 Humble distribution
- Nav2 Stack
- Gazebo Simulator
- Rviz2 Visualization Tool
- Python/C++ programming environment

## Project Structure
ros2_ws/
│
├── src/
│   ├── obstacle_avoidance/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── src/
│   │   └── CMakeLists.txt
│
├── install/
├── build/
└── README.md

## Installation Guide

1. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
2. Install TurtleBot3 and Nav2 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3*
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

4. Clone the project repository:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ryanmaruf615/ros2_obstacle_avoidance.git
   cd ..
   colcon build
   source install/setup.bash

## Launching the System
1. Launch TurtleBot3 simulation:
    ```bash
    ros2 launch nav2_bringup tb3_simulation_launch.py
2. Run the Obstacle Avoidance Node:
    ```bash
    ros2 run obstacle_avoidance avoid_obstacles    
## Future Improvements

Implement advanced path-planning algorithms.

Integrate machine learning-based object recognition.

Improve performance in outdoor environments.

Optimize navigation efficiency using SLAM-based localization.

Deploy and test the algorithm on a real-world TurtleBot3 robot.


## Contributors

Md Maruf Hossain
Moushumi Parvin Tonny

## References

ROS 2 Documentation

Nav2 Stack Documentation

Various academic papers on autonomous navigation and obstacle avoidance
