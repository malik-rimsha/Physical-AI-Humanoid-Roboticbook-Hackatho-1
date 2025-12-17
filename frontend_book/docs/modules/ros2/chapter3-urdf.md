---
sidebar_position: 3
title: "Chapter 3: Humanoid Robot Structure (URDF)"
---

# Chapter 3: Humanoid Robot Structure (URDF)

## Purpose of URDF

Unified Robot Description Format (URDF) is an XML format used to describe the physical and visual properties of robots in ROS. It defines the kinematic and dynamic structure of a robot, including its visual and collision properties, inertial parameters, and joint constraints.

URDF is essential for:
- Robot simulation in tools like Gazebo
- Robot visualization in RViz
- Kinematic analysis and motion planning
- Control algorithm development

## Links, Joints, and Humanoid Body Modeling

### Links
Links represent rigid bodies in the robot structure. Each link has:
- Physical properties (mass, inertia)
- Visual properties (geometry, material, origin)
- Collision properties (geometry for collision detection)

Example link definition:
```xml
<link name="link_name">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints
Joints define the connections between links and their allowed motions. Common joint types include:
- Fixed: No movement allowed
- Revolute: Single axis rotation with limits
- Continuous: Single axis rotation without limits
- Prismatic: Single axis translation with limits
- Floating: Six degrees of freedom
- Planar: Motion on a plane

Example joint definition:
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Humanoid Body Modeling
Humanoid robots require complex URDF models with multiple links and joints to represent the human-like structure. A typical humanoid model includes:
- Torso
- Head with possible degrees of freedom
- Arms with shoulders, elbows, and wrists
- Legs with hips, knees, and ankles
- Hands and feet (optional)

## URDF's Role in Control and Simulation

URDF models are fundamental to robot simulation and control:
- Simulation engines use URDF to create physics models
- Control algorithms use URDF for kinematic calculations
- Motion planning algorithms use URDF for collision checking
- Visualization tools use URDF for robot rendering

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the purpose and structure of URDF files
- Create basic URDF models with links and joints
- Explain how URDF models support robot control and simulation

## Exercises

1. Create a simple URDF model of a 2-link manipulator.
2. Research and explain the difference between URDF and SDF (Simulation Description Format).
3. Find and analyze a URDF file from an open-source humanoid robot project.