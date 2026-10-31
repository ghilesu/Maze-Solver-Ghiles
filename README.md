# Autonomous ROS 2 Maze Solver Robot 🤖🧭

An end-to-end autonomous mobile robot application built for **ROS 2 (Jazzy)** and **Gazebo Simulation**. The project features a custom differential-drive mobile robot equipped with a 2D LiDAR sensor that performs real-time Simultaneous Localization and Mapping (SLAM) via `slam_toolbox`, autonomous environment exploration, and high-speed path planning using **Nav2**.

---

## 📸 Video Demonstration

[![Autonomous Maze Solver in ROS 2 Jazzy](https://img.youtube.com/vi/li1fNgGiies/0.jpg)](https://www.youtube.com/watch?v=li1fNgGiies)

> **Watch on YouTube:** [Autonomous Maze Solving Robot in ROS 2 Jazzy & Gazebo](https://www.youtube.com/watch?v=li1fNgGiies)  
> *The video demonstrates the robot mapping the unknown maze environment in real-time and executing an optimal solution path on its first run.*

---

## 🌟 Key Technical Features

- **Custom Robot Description (URDF / Xacro):** Differential-drive mobile base with wheel joint physics, inertia, and integrated RPLIDAR A2 simulation link (`base_footprint` ➔ `base_link` ➔ `left_wheel` / `right_wheel` / `caster_wheel`).
- **Gazebo Physics World (`maze_world.sdf`):** Multi-corridor simulation maze environment featuring tight turns, dead ends, and obstacle walls.
- **Real-Time SLAM (`slam_toolbox`):** Asynchronous Ceres-solver-based mapping node using `/scan` laser scan data to dynamically build occupancy grid maps (`/map`).
- **Autonomous Navigation & Path Planning (Nav2):** Pre-configured `NavfnPlanner` with A* algorithm capabilities and `MPPIController` tuned for obstacle avoidance and dynamic trajectory evaluation.
- **Custom Autonomous Solver Node (`maze_solver.py`):** Python-based decision-making node that coordinates mapping exploration and autonomous goal targeting.
- **RViz2 Visualization Suite (`maze_view.rviz`):** Real-time monitoring configuration pre-loaded with Map layer, Global Costmap, Local Costmap, Robot Model, and active LaserScans.

---

## 🛠️ System Requirements & Dependencies

- **Operating System:** Ubuntu 24.04 LTS (or Ubuntu 22.04 LTS) / Windows (via WSL2 or Git Bash)
- **ROS Distribution:** ROS 2 Jazzy Jalisco
- **Required Packages:**
  ```bash
  sudo apt update
  sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui
  ```

---

## 🏗️ Project Architecture & File Structure

```text
maze_ws/
└── src/
    └── maze_robot/
        ├── config/
        │   ├── maze_params.yaml    # Nav2 Costmap & MPPI Controller parameters
        │   └── slam_params.yaml    # SLAM Toolbox Ceres solver & frame mappings
        ├── launch/
        │   ├── spawn.launch.py     # Launches Gazebo & spawns URDF model
        │   ├── slam.launch.py      # Starts asynchronous SLAM Toolbox node
        │   ├── rviz.launch.py      # Opens RViz with pre-configured display layers
        │   └── solve_maze.launch.py# Main autonomous solver execution launch
        ├── rviz/
        │   └── maze_view.rviz      # Saved RViz layout display configuration
        ├── src/
        │   └── maze_solver.py      # Custom Python exploration & navigation node
        ├── urdf/
        │   └── maze_robot.urdf.xacro# Robot kinematic description & sensor frames
        ├── worlds/
        │   └── maze_world.sdf      # Gazebo environment layout definition
        ├── CMakeLists.txt          # Build configuration script
        └── package.xml             # Package metadata & ROS dependencies
```

---

## 🚀 Installation & Build Instructions

1. **Clone the Repository into your Workspace:**
   ```bash
   mkdir -p ~/maze_ws/src
   cd ~/maze_ws/src
   git clone https://github.com/ghilesu/Maze-Solver-Ghiles.git maze_robot
   ```

2. **Build the Package:**
   ```bash
   cd ~/maze_ws
   colcon build --packages-select maze_robot
   ```

3. **Source Workspace Environment:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

---

## 🏃 Launch Sequence

To run the complete system, open separate terminal windows, navigate to your workspace, source the setup files, and run the following launch commands:

### Terminal 1: Launch Gazebo Simulation World
Spawns the `maze_robot` inside the custom Gazebo environment:
```bash
cd ~/maze_ws && source install/setup.bash
ros2 launch maze_robot spawn.launch.py
```

### Terminal 2: Start SLAM Toolbox (Mapping)
Initiates dynamic laser-scan mapping:
```bash
cd ~/maze_ws && source install/setup.bash
ros2 launch maze_robot slam.launch.py
```

### Terminal 3: Start RViz2 Visualization
Displays live map accumulation, costmaps, and robot transforms:
```bash
cd ~/maze_ws && source install/setup.bash
ros2 launch maze_robot rviz.launch.py
```

### Terminal 4: Execute Autonomous Solver
Launches the exploration node to solve the maze:
```bash
cd ~/maze_ws && source install/setup.bash
ros2 run maze_robot maze_solver.py
```

---

## 📡 Topic & Transform Summary

| Frame / Topic | Description | Type / Parent |
| :--- | :--- | :--- |
| **`map`** | Global fixed frame published by `slam_toolbox` | Transform Root |
| **`odom`** | Odometry frame published by differential drive controller | Parent: `map` |
| **`base_footprint`** | Ground projection link of the robot | Parent: `odom` |
| **`base_link`** | Chassis center reference link | Parent: `base_footprint` |
| **`/scan`** | 2D LiDAR range measurement topic | `sensor_msgs/msg/LaserScan` |
| **`/map`** | Real-time occupancy grid topic | `nav_msgs/msg/OccupancyGrid` |
| **`/cmd_vel`** | Velocity command topic for movement | `geometry_msgs/msg/Twist` |

---

## 📜 License & Maintainer

- **Maintainer:** Ghiles (`gmameri69@gmail.com`)
- **Package Name:** `maze_robot`
- **ROS 2 Version:** Jazzy Jalisco
```

