# Isaac Module Quick Reference

## Key Commands

### Isaac Sim
- Launch Isaac Sim: `./isaac-sim.python.sh --config=standalone_physics_config.json`
- Isaac Sim documentation: Available in the Isaac Sim installation directory

### Isaac ROS
- Perception pipeline: `ros2 launch isaac_ros_examples perception_pipeline.launch.py`
- VSLAM system: `ros2 launch isaac_ros_examples vslam.launch.py`
- Navigation system: `ros2 run nav2_bringup nav2_launch.py`

### ROS 2 Integration
- List topics: `ros2 topic list`
- Monitor topic: `ros2 topic echo <topic_name>`
- Check services: `ros2 service list`

## Common File Locations
- Isaac Sim: `/opt/nvidia/isaac_sim`
- Isaac ROS packages: ROS 2 workspace with Isaac ROS packages installed
- Isaac Lab: Included with Isaac Sim installation

## Configuration Files
- Simulation config: `standalone_physics_config.json`
- ROS launch files: In `isaac_ros_examples` package
- Navigation config: In Nav2 configuration directory

## GPU Requirements
- Minimum: NVIDIA GPU with CUDA capability 6.0+
- Recommended: RTX series for optimal performance
- VRAM: Minimum 8GB for complex simulations