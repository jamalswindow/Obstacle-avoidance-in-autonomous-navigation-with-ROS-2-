# Obstacle Avoidance in Autonomous Navigation with ROS 2

**Project Group:** OBS-1
**Student:** Md Jamal Uddin (1387201)
**Email:** md.uddin2@stud.fra-uas.de
**Course:** Autonomous Intelligent Systems (AIS), Frankfurt University of Applied Sciences
**Supervisor:** Prof. Dr. Peter Nauth

## Overview

This project presents a comparative evaluation of four obstacle avoidance strategies for autonomous mobile robots using the TurtleBot3 Burger platform in a Gazebo simulation environment with ROS 2 Humble. The goal is to analyze and compare the performance of different local planning approaches in terms of safety, efficiency, and adaptability.

### Approaches

| # | Approach | Type | Nav2 | Description |
|---|----------|------|------|-------------|
| 1 | **Reactive** | Custom node | No | 5-zone LiDAR-based obstacle avoidance with anti-oscillation cooldown |
| 2 | **DWB** | Nav2 plugin | Yes | Dynamic Window Approach B - samples velocity space and scores trajectories |
| 3 | **TEB** | Nav2 plugin | Yes | Timed Elastic Band - optimizes trajectory as elastic band with obstacle constraints |
| 4 | **MPPI** | Nav2 plugin | Yes | Model Predictive Path Integral - stochastic sampling of 1000 trajectories |

### Metrics Collected

Each approach is evaluated using a metrics logger that records:
- **Minimum clearance** from obstacles (m)
- **Average clearance** from obstacles (m)
- **Linear and angular velocity** profiles
- **Distance traveled** (m)
- **Collision count** (clearance < 0.12m robot radius)
- **Recovery events** (backup maneuvers)
- **Rotation events** (spin recoveries)

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- TurtleBot3 packages: `ros-humble-turtlebot3*`
- Nav2 navigation stack: `ros-humble-navigation2`, `ros-humble-nav2-bringup`
- Gazebo Classic simulator
- g2o solver library: `ros-humble-libg2o`

```bash
# Install system dependencies
sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-libg2o
```

## Build

The TEB local planner and its costmap_converter dependency are included in the source tree and must be built from source.

```bash
# Clone the repository
git clone https://github.com/jamalswindow/Obstacle-avoidance-in-autonomous-navigation-with-ROS-2-.git
cd Obstacle-avoidance-in-autonomous-navigation-with-ROS-2-/ros2_ws

# Build TEB dependencies first, then the main package
colcon build --packages-select costmap_converter_msgs costmap_converter teb_msgs teb_local_planner
source install/setup.bash
colcon build --packages-select obstacle_avoidance_tb3
source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
```

## Running Tests

Each test requires two terminals. Always start Gazebo first, then launch the desired approach.

### Terminal 1: Start Gazebo Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: Launch an Approach

**Reactive obstacle avoidance:**
```bash
ros2 launch obstacle_avoidance_tb3 laser_scan.launch.py
```

**DWB (Dynamic Window Approach B):**
```bash
ros2 launch obstacle_avoidance_tb3 dwb_navigation.launch.py
```

**TEB (Timed Elastic Band):**
```bash
ros2 launch obstacle_avoidance_tb3 teb_navigation.launch.py
```

**MPPI (Model Predictive Path Integral):**
```bash
ros2 launch obstacle_avoidance_tb3 mppi_navigation.launch.py
```

### How It Works

- **Reactive:** Directly publishes velocity commands based on LiDAR zones. No map or path planning needed.
- **DWB / TEB / MPPI:** Each launch file starts the full Nav2 stack (AMCL localization, global planner, local controller, behavior tree navigator, costmaps, recovery behaviors). An automatic **waypoint sender** node waits for Nav2 to become active, then sends 10 navigation goals in a loop around the TurtleBot3 World map. A **metrics logger** node records performance data to CSV files.

### Manual Goal Sending (Alternative)

Shell scripts are provided for manually sending goals without the automatic waypoint sender:

```bash
# Send a single goal
bash send_goal.sh

# Send goals in a continuous loop
bash send_goals_loop.sh
```

## Results

Test results are saved in the `results/` directory as CSV files and summary text files, named by approach and timestamp:
```
results/
  metrics_reactive_20260312_162729.csv
  metrics_reactive_20260312_162729_summary.txt
  metrics_dwb_20260312_171633.csv
  metrics_dwb_20260312_171633_summary.txt
  metrics_teb_20260312_171242.csv
  metrics_teb_20260312_171242_summary.txt
  metrics_mppi_20260312_170747.csv
  metrics_mppi_20260312_170747_summary.txt
  ...
```

## Project Structure

```
.
├── README.md
├── .gitignore
├── send_goal.sh                          # Manual single goal script
├── send_goals_loop.sh                    # Manual goal loop script
├── results/                              # Test metrics (CSV + summaries)
└── ros2_ws/src/
    ├── obstacle_avoidance_tb3/           # Main package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── config/
    │   │   ├── dwa_config.yaml           # DWB full Nav2 stack config
    │   │   ├── teb_config.yaml           # TEB full Nav2 stack config
    │   │   └── mppi_config.yaml          # MPPI full Nav2 stack config
    │   ├── launch/
    │   │   ├── laser_scan.launch.py      # Reactive approach
    │   │   ├── dwb_navigation.launch.py  # DWB with Nav2
    │   │   ├── teb_navigation.launch.py  # TEB with Nav2
    │   │   └── mppi_navigation.launch.py # MPPI with Nav2
    │   └── obstacle_avoidance_tb3/
    │       ├── __init__.py
    │       ├── obstacle_avoidance.py     # Reactive 5-zone LiDAR node
    │       ├── waypoint_sender.py        # Automatic Nav2 goal sender
    │       └── metrics_logger.py         # Performance metrics logger
    ├── costmap_converter/                # TEB dependency (built from source)
    │   ├── costmap_converter/
    │   └── costmap_converter_msgs/
    └── teb_local_planner/                # TEB planner (built from source)
        ├── teb_local_planner/
        └── teb_msgs/
```

## Configuration Details

### Robot Parameters (TurtleBot3 Burger)
| Parameter | Value |
|-----------|-------|
| Max linear velocity | 0.22 m/s |
| Max angular velocity | 2.84 rad/s |
| Linear acceleration limit | 1.5 m/s^2 |
| Angular acceleration limit | 2.0 rad/s^2 |
| Robot radius | 0.105 m (TEB) / 0.15 m (DWB, MPPI costmaps) |

### Nav2 Stack (DWB, TEB, MPPI)
- **Localization:** AMCL (Adaptive Monte Carlo Localization)
- **Global Planner:** NavfnPlanner with A* search
- **Behavior Tree:** `navigate_to_pose_w_replanning_and_recovery.xml`
- **Recovery Behaviors:** Spin, Backup, Wait
- **Costmap Layers:** Static, Obstacle (2D LiDAR), Inflation

### Key Tuning Notes
- `transform_tolerance: 5.0` and `bond_timeout: 60.0` are set high to accommodate VirtualBox simulation with low real-time factor
- MPPI `controller_frequency: 10.0` must be >= `1/model_dt` (1/0.1 = 10 Hz)
- DWB/TEB use `controller_frequency: 3.0` for stability in slow simulation
- Angular acceleration limited to 2.0 rad/s^2 to prevent TurtleBot3 Burger from flipping
- Waypoint sender uses `stamp=0` to avoid wall-clock vs sim-time mismatch

## Tools and Technologies

- **ROS 2 Humble** - Robot Operating System
- **Nav2** - Navigation 2 stack (DWB, TEB, MPPI controllers)
- **Gazebo Classic** - Physics simulation
- **TurtleBot3 Burger** - Differential drive mobile robot
- **AMCL** - Adaptive Monte Carlo Localization
- **Python 3** - Node implementation (rclpy)
- **C++** - TEB and costmap_converter (built from source)
