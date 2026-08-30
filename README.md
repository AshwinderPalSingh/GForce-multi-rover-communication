# GForce Multi-Rover Communication — ROS 2 (Humble) + Ignition Gazebo

A multi-rover hospital delivery system with **intelligent goal assignment and adaptive switching**. Built on ROS 2 Humble and Ignition Gazebo (Fortress). When a goal is published, the nearest rover is automatically assigned. If that rover gets blocked, the system switches to the other rover. If the original rover's path later clears and it has a significant distance advantage, the system re-switches back.

> **Fork of** [bismanroopsingh/Ros-project](https://github.com/bismanroopsingh/Ros-project) — extended from single-robot delivery to multi-rover intelligent coordination.

---

## Demo

> Two rovers navigating a hospital environment with automatic goal assignment, stuck detection, and adaptive switching.

---

## Table of Contents

- [Features](#features)
- [Multi-Rover Architecture](#multi-rover-architecture)
- [Goal Assignment Logic](#goal-assignment-logic)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [How Goal Assignment Works](#how-goal-assignment-works)
- [Configuration](#configuration)
- [Key Locations](#key-locations)
- [Known Issues & Fixes](#known-issues--fixes)
- [Troubleshooting](#troubleshooting)

---

## Features

- **Two identical differential-drive rovers** operating simultaneously in the same hospital world
- **Intelligent goal assignment**: nearest rover is automatically assigned incoming goals
- **Stuck detection**: monitors rover progress and detects when a rover is blocked
- **Adaptive switching**: if the assigned rover is stuck, the goal is handed off to the backup rover
- **Re-switch with distance threshold**: if the originally-blocked rover's path clears AND it has a >50% distance advantage, the goal is re-assigned back
- Custom differential-drive robot with 2D LiDAR (12m range, 360°)
- L-shaped hospital world with patient rooms, nurses station, reception, and charging bay
- Nav2 with A\* global planner, DWB local controller, AMCL localization, EKF odometry
- Fully namespaced multi-robot support (separate TF trees, costmaps, controllers per rover)

---

## Multi-Rover Architecture

```
User publishes /goal_pose (geometry_msgs/PoseStamped)
         │
         ▼
 ┌───────────────────────────────────────┐
 │   goal_assignment_manager.py          │  ← The Brain
 │                                       │
 │   Subscribes:                         │
 │     /goal_pose       (incoming goal)  │
 │     /rover1/odom     (rover1 pos)     │
 │     /rover2/odom     (rover2 pos)     │
 │                                       │
 │   Action Clients:                     │
 │     /rover1/navigate_to_pose          │
 │     /rover2/navigate_to_pose          │
 │                                       │
 │   Logic:                              │
 │     1. Assign to nearest rover        │
 │     2. Monitor progress (stuck?)      │
 │     3. Switch if blocked              │
 │     4. Re-switch if path clears       │
 └──────────────┬────────────────────────┘
                │
        ┌───────┴───────┐
        ▼               ▼
 ┌─────────────┐ ┌─────────────┐
 │  Rover 1    │ │  Rover 2    │
 │  Nav2 Stack │ │  Nav2 Stack │
 │  /rover1/*  │ │  /rover2/*  │
 └─────────────┘ └─────────────┘
```

Each rover has its own fully independent Nav2 stack:
- AMCL localization (different initial pose per rover)
- A\* global planner
- DWB local controller
- Local + Global costmaps
- Behavior tree navigator
- EKF odometry fusion

---

## Goal Assignment Logic

### State Machine

```
             ┌─────────────────────────────────┐
             │   GOAL PUBLISHED (by user/RViz) │
             └────────────────┬────────────────┘
                              │
                   Compute distances to goal
                   from Rover1 & Rover2
                              │
                    Assign to NEAREST rover
                              │
              ┌───────────────▼──────────────────┐
              │   ASSIGNED ROVER → NAVIGATING     │
              └───────────────┬──────────────────┘
                              │
              ┌───────────────▼──────────────────┐
              │  Check progress every 3s          │
              │  - Is rover making progress?       │
              │  - Is Nav2 reporting a FAILURE?    │
              └───────────────┬──────────────────┘
                  Blocked?    │     Making progress?
          ┌─────────────────┐ │ ┌──────────────────────┐
          │  SWITCH to other│ │ │  Continue navigating  │
          │  rover          │ │ │  → check for re-switch│
          └────────┬────────┘ │ └──────────────────────┘
                   │          │
     ──────────────▼───────────────────────────────────
     While backup rover moves toward goal:
       - Keep monitoring blocked rover's path
       - If blocked rover's path opens AND
         distance_advantage > 50%:
             Re-assign to originally-blocked rover
       - If advantage ≤ threshold → stay with current
     ────────────────────────────────────────────────────
```

### Stuck Detection

Every 3 seconds the manager records the assigned rover's position. If the rover moves less than **0.05m** over **3 consecutive checks** (9 seconds total), it is declared **BLOCKED** and the goal switches to the other rover.

### Re-Switch Threshold

While the backup rover navigates, the manager checks if the originally-blocked rover can now reach the goal. A re-switch happens **only if**:
1. The original rover's path is clear (verified via `ComputePathToPose`)
2. The original rover's distance to goal is **less than 50%** of the backup rover's current distance

This prevents unnecessary ping-ponging between rovers.

---

## Project Structure

```
hospital_robot_description/
├── config/
│   ├── nav2_params.yaml              # Original Nav2 config (single robot)
│   ├── nav2_params_rover1.yaml       # Nav2 config for rover1 (namespaced)
│   ├── nav2_params_rover2.yaml       # Nav2 config for rover2 (namespaced)
│   ├── slam_toolbox_params.yaml      # SLAM mapping configuration
│   └── navigate_w_replanning.xml     # Custom BT for recovery
├── launch/
│   ├── gz_gazebo.launch.py           # Gazebo + 2 rovers + bridges
│   ├── multi_rover_nav2.launch.py    # 2× Nav2 stacks + goal manager
│   ├── slam.launch.py                # SLAM mapping
│   └── nav2.launch.py                # Original single-robot Nav2
├── maps/
│   ├── hospital_map.pgm              # Saved occupancy grid map
│   └── hospital_map.yaml             # Map metadata
├── meshes/
│   └── *.stl                         # Robot 3D models
├── scripts/
│   ├── goal_assignment_manager.py    # ★ Multi-rover brain
│   ├── cmd_vel_relay.py              # Relay nav2 cmd_vel (namespaced)
│   ├── recovery_helper.py            # Manual costmap clearing
│   ├── pd_teleop.py                  # PD-controlled teleoperation
│   └── odom_tf_publisher.py          # Odometry TF publisher
├── urdf/
│   └── robot.urdf.xacro              # Robot description (parameterized)
├── worlds/
│   └── hospital.world                # Hospital SDF world
├── CMakeLists.txt
└── package.xml
```

---

## Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Ignition Gazebo | Fortress |
| Ubuntu | 22.04 |

### Required ROS 2 packages

```bash
sudo apt update && sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard
```

---

## Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/AshwinderPalSingh/GForce-multi-rover-communication.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Quick Start

### Launch Multi-Rover System

```bash
# Terminal 1 — Launch Gazebo with both rovers
ros2 launch hospital_robot_description gz_gazebo.launch.py

# Terminal 2 — Launch Nav2 for both rovers + goal assignment manager
ros2 launch hospital_robot_description multi_rover_nav2.launch.py

# Terminal 3 — Open RViz
rviz2
```

### Send a Goal

```bash
# Via terminal — send goal to the supply table
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: -8.5, y: 8.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

Or use RViz's **Nav2 Goal** button to click a destination on the map.

### What Happens

1. The goal assignment manager receives the goal
2. It computes distances from both rovers to the goal
3. The **nearest rover** is assigned the goal
4. If that rover gets **stuck** (no movement for 9s), the goal **switches** to the other rover
5. If the blocked rover's path **clears** and it's **>50% closer** to the goal, it **re-switches back**

---

## Configuration

### Goal Assignment Parameters

Set in `launch/multi_rover_nav2.launch.py`:

| Parameter | Default | Description |
|---|---|---|
| `rover_names` | `['rover1', 'rover2']` | List of rover namespaces |
| `progress_check_interval` | `3.0` s | How often to check progress |
| `stuck_distance_threshold` | `0.05` m | Movement below this = stuck |
| `stuck_consecutive_checks` | `3` | Checks needed to declare blocked (3 × 3s = 9s) |
| `reswitch_distance_ratio` | `0.5` | Re-assign only if blocked rover < 50% distance |
| `goal_tolerance` | `0.5` m | Distance to consider goal reached |

### Rover Spawn Positions

| Rover | x | y | Description |
|---|---|---|---|
| rover1 | 0.0 | -2.9 | Main corridor — original spawn point |
| rover2 | 3.0 | -2.9 | Main corridor — 3m east of rover1 |

---

## Key Locations

| Location | x | y | Notes |
|---|---|---|---|
| Rover1 Spawn | 0.0 | -2.9 | Main corridor |
| Rover2 Spawn | 3.0 | -2.9 | Main corridor |
| **Supply Table** | **-8.5** | **8.0** | Delivery Stop 1 |
| **Reception Desk** | **-8.5** | **7.5** | Delivery Stop 2 |
| Supply Boxes | -8.5 | 10.5 | Near north wall |
| Nurses Station | 1.0 | -0.5 | In front of desk |
| Charging Dock | -8.5 | -4.5 | Storage bay |
| Room 1 Doorway | 0.0 | 3.0 | Outside patient room |
| Room 2 Doorway | 6.0 | 3.0 | Outside patient room |

---

## Known Issues & Fixes

### Robot stops near obstacles
**Cause:** Inflation radius too large — adjacent zones merge.
**Fix:** `inflation_radius: 0.05` in both costmaps.

### Nav2 BT path was hardcoded
**Cause:** Original config had `/home/bismanroop/...` hardcoded path.
**Fix:** Removed hardcoded path — Nav2 uses its default BT.

### Two rovers collide during spawn
**Cause:** Rovers spawned too close together.
**Fix:** Rover2 spawns 3m east and 2s after Rover1.

### Costmap not clearing stale obstacles
**Cause:** `observation_persistence > 0`.
**Fix:** Set to `0.0` in both rover param files.

---

## Troubleshooting

### Check both rovers are alive
```bash
ros2 topic list | grep -E "rover1|rover2"
ros2 topic hz /rover1/scan  # expect ~10 Hz
ros2 topic hz /rover2/scan  # expect ~10 Hz
```

### Check TF tree for both rovers
```bash
ros2 run tf2_tools view_frames
# Expected: map → rover1/odom → rover1/base_link → rover1/lidar_link
#           map → rover2/odom → rover2/base_link → rover2/lidar_link
```

### Check goal assignment manager
```bash
ros2 node info /goal_assignment_manager
ros2 topic echo /goal_pose --once
```

### Manually clear costmaps
```bash
# Rover 1
ros2 service call /rover1/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}

# Rover 2
ros2 service call /rover2/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
```

---

## Original Single-Robot Mode

The original single-robot mode is still available:

```bash
# Terminal 1 — Gazebo (single robot)
# Note: gz_gazebo.launch.py now spawns 2 rovers by default
# For single robot, use the original nav2.launch.py which works with rover1

ros2 launch hospital_robot_description gz_gazebo.launch.py
ros2 launch hospital_robot_description nav2.launch.py
```

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Acknowledgements

- [bismanroopsingh/Ros-project](https://github.com/bismanroopsingh/Ros-project) — original single-robot codebase
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)