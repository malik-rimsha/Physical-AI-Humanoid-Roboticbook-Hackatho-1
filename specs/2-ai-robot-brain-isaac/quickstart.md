# Quickstart: AI-Robot Brain (NVIDIA Isaac™)

## Overview
This module introduces NVIDIA Isaac as the AI brain for humanoid robots, focusing on perception, navigation, and training. The module covers Isaac overview, perception (VSLAM), and humanoid navigation for AI/software students.

## Prerequisites
- Ubuntu 20.04/22.04 LTS
- NVIDIA GPU with CUDA capability 6.0+ (recommended RTX series)
- 16GB RAM minimum (32GB recommended)
- ROS 2 (Humble Hawksbill or later)
- NVIDIA Isaac Sim
- Isaac ROS packages
- Git for version control

## Setup Instructions

### 1. Install NVIDIA Isaac Prerequisites
```bash
# Install NVIDIA drivers and CUDA
sudo apt update
sudo apt install nvidia-driver-535 nvidia-utils-535

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key add /var/cuda-repo-ubuntu2004/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt update
sudo apt install cuda-toolkit-12-3
```

### 2. Install Isaac Sim
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow installation instructions for your system
# Typically installed in /opt/nvidia/isaac_sim
```

### 3. Install Isaac ROS
```bash
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common ros-humble-isaac-ros-perception ros-humble-isaac-ros-nav2

# Install additional dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 4. Set up the Workspace
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws
# Clone Isaac ROS examples
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_examples.git src/isaac_ros_examples

colcon build
source install/setup.bash
```

## Getting Started with the Module

### Chapter 1: NVIDIA Isaac and AI-Driven Robotics
1. Launch Isaac Sim with basic environment:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.python.sh --config=standalone_physics_config.json
   ```
2. Explore Isaac's simulation capabilities
3. Learn about synthetic data generation

### Chapter 2: Isaac ROS and Perception
1. Set up perception pipeline with Isaac ROS:
   ```bash
   ros2 launch isaac_ros_examples perception_pipeline.launch.py
   ```
2. Implement VSLAM for localization and mapping
3. Test perception algorithms with synthetic data

### Chapter 3: Navigation and Motion Planning
1. Configure Nav2 for humanoid navigation:
   ```bash
   ros2 launch isaac_ros_examples humanoid_nav2.launch.py
   ```
2. Implement path planning considering bipedal constraints
3. Integrate perception with navigation

## Key Commands
- `ros2 launch isaac_ros_examples perception_pipeline.launch.py` - Start perception pipeline
- `ros2 launch isaac_ros_examples vslam.launch.py` - Launch VSLAM system
- `ros2 run nav2_bringup nav2_launch.py` - Start navigation system
- `isaac-sim` - Launch Isaac Sim application
- `ros2 topic list` - View available ROS 2 topics
- `ros2 topic echo <topic_name>` - Monitor topic data

## Next Steps
1. Complete Chapter 1: NVIDIA Isaac and AI-Driven Robotics
2. Move to Chapter 2: Isaac ROS and Perception
3. Finish with Chapter 3: Navigation and Motion Planning
4. Integrate all concepts in a comprehensive humanoid robot project