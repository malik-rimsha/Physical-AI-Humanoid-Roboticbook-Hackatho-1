# Quickstart: Digital Twin Simulation for Humanoid Robots

## Overview
This module teaches physics-based simulation and digital twin concepts for humanoid robots using Gazebo and Unity, with integration to ROS 2 systems.

## Prerequisites
- Ubuntu 20.04/22.04 LTS or Windows 10/11
- 8GB RAM minimum (16GB recommended)
- ROS 2 (Humble Hawksbill or later)
- Gazebo Garden or Fortress
- Unity Hub with Unity 2022.3 LTS
- Git for version control

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
# For ROS 2 and Gazebo
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs

# For Unity (if needed for high-fidelity rendering)
# Download from Unity Hub
```

### 3. Set up the Workspace
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

## Getting Started with the Module

### Chapter 1: Digital Twins and Physics Simulation
1. Launch Gazebo with a basic humanoid robot:
   ```bash
   ros2 launch digital_twin_examples physics_simulation.launch.py
   ```
2. Observe how gravity affects the robot
3. Interact with the environment to see collision responses

### Chapter 2: Environment & Interaction Simulation
1. Build custom environments in Gazebo:
   ```bash
   ros2 launch digital_twin_examples environment_builder.launch.py
   ```
2. Import the same environment into Unity for high-fidelity rendering
3. Test human-robot interaction scenarios

### Chapter 3: Sensor Simulation
1. Configure virtual sensors on the humanoid robot:
   ```bash
   ros2 launch digital_twin_examples sensor_simulation.launch.py
   ```
2. Observe sensor data streams in ROS 2:
   ```bash
   ros2 topic echo /robot/lidar_scan sensor_msgs/msg/LaserScan
   ```
3. Connect sensor data to perception pipelines

## Key Commands
- `ros2 run gazebo_ros gazebo` - Launch Gazebo simulation
- `ros2 launch <package> <launch_file>` - Run simulation scenarios
- `rviz2` - Visualize robot state and sensor data
- `ros2 topic list` - View available ROS 2 topics
- `ros2 topic echo <topic_name>` - Monitor topic data

## Next Steps
1. Complete Chapter 1: Digital Twins and Physics Simulation
2. Move to Chapter 2: Environment & Interaction Simulation
3. Finish with Chapter 3: Sensor Simulation
4. Integrate all concepts in a comprehensive project