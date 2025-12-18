# Quickstart: Vision-Language-Action (VLA) Integration

## Overview
This module introduces Vision-Language-Action (VLA) systems for humanoid robotics, focusing on how language, vision, and action combine to enable autonomous behavior. The module covers VLA foundations, voice-to-action mapping with OpenAI Whisper, and LLM-based cognitive planning for humanoid robots.

## Prerequisites
- Ubuntu 20.04/22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.8+ with pip
- OpenAI API access (for Whisper examples)
- Git for version control
- Basic knowledge of ROS 2 concepts

## Setup Instructions

### 1. Install ROS 2 Humble
```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

### 2. Install Whisper Dependencies
```bash
# Install Python dependencies
pip3 install openai-whisper
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install sounddevice pyaudio
```

### 3. Set up Workspace
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/vla_ws/src
cd ~/vla_ws

# Install ROS 2 VLA example packages
git clone https://github.com/ros2/examples.git src/examples
git clone https://github.com/ros2/common_interfaces.git src/common_interfaces

colcon build
source install/setup.bash
```

## Getting Started with the Module

### Chapter 1: VLA Foundations
1. Understand the core concepts of Vision-Language-Action systems
2. Learn about the integration of perception, language, and action
3. Explore how LLMs enable embodied intelligence in robotics
4. Review the architecture patterns for VLA systems

### Chapter 2: Voice-to-Action Mapping
1. Set up OpenAI Whisper for voice command processing:
   ```bash
   # Test Whisper installation
   python3 -c "import whisper; print(whisper.available_models())"
   ```
2. Implement voice command processing pipeline
3. Map natural language to robot intents
4. Connect to ROS 2 action execution

### Chapter 3: Cognitive Planning
1. Configure LLM-based task planning system
2. Implement multi-step action sequencing
3. Test autonomous humanoid execution scenarios
4. Integrate safety validation layers

## Key Commands
- `ros2 run demo_nodes_cpp talker` - Test ROS 2 communication
- `python3 -m speech_recognition` - Test audio input capabilities
- `ros2 action list` - View available ROS 2 actions
- `ros2 service list` - View available ROS 2 services
- `python3 -c "import whisper; model = whisper.load_model('base'); result = model.transcribe('audio.wav'); print(result['text'])"` - Test Whisper transcription

## Next Steps
1. Complete Chapter 1: VLA Foundations
2. Move to Chapter 2: Voice-to-Action Mapping
3. Finish with Chapter 3: Cognitive Planning and Autonomy
4. Integrate all concepts in a comprehensive VLA system project