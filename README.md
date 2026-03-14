# Obstacle Avoidance in Autonomous Navigation with ROS 2

A comparative study of obstacle avoidance strategies for autonomous navigation using **TurtleBot3** in ROS 2 Humble. This project evaluates three approaches:

1. **Reactive Obstacle Avoidance** — Direct LiDAR-based avoidance using scan data
2. **DWB (Dynamic Window Approach)** — Nav2's default local planner
3. **TEB (Timed Elastic Band)** — Optimization-based local planner from `teb_local_planner`

## Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble**
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_simulations`)
- Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- Gazebo (Classic)
- [g2o](https://github.com/RainerKuemmerle/g2o) (required by TEB planner)

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2

# Set TurtleBot3 model
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

## Project Structure

```
.
├── obstacle_avoidance_tb3/       # Main ROS 2 Python package
│   ├── obstacle_avoidance.py     # Reactive obstacle avoidance node
│   └── navigation_metrics.py     # Automated benchmark data collector
├── config/
│   ├── dwa_config.yaml           # Standalone DWA parameters
│   ├── nav2_dwb_params.yaml      # Full Nav2 stack config (DWB planner)
│   └── nav2_teb_params.yaml      # Full Nav2 stack config (TEB planner)
├── launch/
│   ├── laser_scan.launch.py      # Launch reactive avoidance
│   ├── navigation.launch.py      # Basic Nav2 navigation launch
│   └── planner_comparison.launch.py  # Launch with DWB or TEB selection
├── costmap_converter/            # Costmap converter plugin (TEB dependency)
├── teb_local_planner/            # TEB local planner (humble-devel)
├── results/
│   └── metrics.csv               # Benchmark results (DWB & TEB)
├── package.xml
├── setup.py
└── setup.cfg
```

## Building

```bash
# Create workspace and clone
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/jamalswindow/Obstacle-avoidance-in-autonomous-navigation-with-ROS-2-.git

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running

### 1. Reactive Obstacle Avoidance

Runs the robot with direct LiDAR-based obstacle avoidance (no Nav2 stack):

```bash
# Terminal 1: Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch reactive avoidance
ros2 launch obstacle_avoidance_tb3 laser_scan.launch.py
```

### 2. Navigation with DWB Planner

```bash
ros2 launch obstacle_avoidance_tb3 planner_comparison.launch.py planner:=dwb
```

### 3. Navigation with TEB Planner

```bash
ros2 launch obstacle_avoidance_tb3 planner_comparison.launch.py planner:=teb
```

### 4. Running Benchmarks

Collect navigation metrics automatically across 5 predefined goal poses:

```bash
# Start the simulation and Nav2 stack first (using either planner)
ros2 launch obstacle_avoidance_tb3 planner_comparison.launch.py planner:=dwb

# In another terminal, run the metrics collector
ros2 run obstacle_avoidance_tb3 navigation_metrics --ros-args \
    -p planner_name:=dwb \
    -p output_file:=$HOME/ros2_ws/results/metrics.csv
```

## Configuration

| File | Description |
|------|-------------|
| `config/dwa_config.yaml` | Standalone DWA planner parameters (velocity, acceleration, tolerances) |
| `config/nav2_dwb_params.yaml` | Complete Nav2 stack config using DWB local planner |
| `config/nav2_teb_params.yaml` | Complete Nav2 stack config using TEB local planner |

Both Nav2 configs include: AMCL localization, behavior tree navigator, controller server, local/global costmaps, planner server, behavior server, waypoint follower, and velocity smoother.

### Key TEB Parameters

- `min_obstacle_dist: 0.15` — Minimum distance to obstacles
- `enable_homotopy_class_planning: true` — Explores multiple trajectory classes
- `costmap_converter_plugin: CostmapToPolygonsDBSMCCH` — Converts costmap to polygons for TEB

## Benchmark Results

Results from automated tests using 5 goal poses in the TurtleBot3 Gazebo world:

| Metric | DWB | TEB |
|--------|-----|-----|
| Success Rate | 8/10 (80%) | 5/5 (100%) |
| Avg Time (succeeded) | 33.2 s | 113.0 s |
| Avg Path Length | 2.3 m | 5.5 m |
| Min Clearance | 0.15 m | 0.29 m |
| Total Collisions | 0 | 0 |

> **Note:** TEB's higher average time is skewed by one outlier run (Goal 1: 464s). Excluding that run, TEB averages ~25s per goal — faster than DWB's average. TEB also maintains better obstacle clearance (0.29m vs 0.15m minimum).

Full results are available in [`results/metrics.csv`](results/metrics.csv).

## Author

**Md Jamal Uddin**
Frankfurt University of Applied Sciences
md.uddin2@stud.fra-uas.de
