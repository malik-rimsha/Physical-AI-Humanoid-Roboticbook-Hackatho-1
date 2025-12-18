---
sidebar_position: 2
title: "Chapter 2: Environment & Interaction Simulation"
---

# Chapter 2: Environment & Interaction Simulation

## Building Robot Environments in Gazebo

Creating realistic environments is crucial for testing robotic systems. Gazebo provides powerful tools for building complex environments that accurately represent real-world scenarios.

### Environment Components

A typical robot environment consists of:

- **Terrain**: Ground surfaces with different properties (grass, concrete, sand)
- **Obstacles**: Static and dynamic objects that robots must navigate around
- **Interactive elements**: Objects that robots can manipulate or interact with
- **Lighting**: Proper illumination for visual sensors
- **Atmospheric effects**: Weather conditions, fog, etc.

### World File Structure

Gazebo environments are defined in SDF (Simulation Description Format) files:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include models from Gazebo Model Database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom models -->
    <model name="my_obstacle">
      <pose>1 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Creating Static Environments

Static environments are the most common type of simulation environment. They include:

1. **Building layouts**: Rooms, corridors, doorways
2. **Outdoor spaces**: Parks, streets, industrial areas
3. **Specialized environments**: Factories, warehouses, laboratories

#### Building Layouts

To create building layouts in Gazebo:

1. Use the **Building Editor** plugin in Gazebo GUI
2. Define rooms with walls, doors, and windows
3. Add furniture and fixtures as needed
4. Set appropriate material properties for surfaces

#### Model Composition

Complex environments are built by composing simpler models:

```xml
<!-- Create a room with furniture -->
<model name="office_room">
  <include>
    <uri>model://wall_segment</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>
  <include>
    <uri>model://desk</uri>
    <pose>2 1 0 0 0 0</pose>
  </include>
  <include>
    <uri>model://chair</uri>
    <pose>2.5 0.5 0 0 0 1.57</pose>
  </include>
</model>
```


### Dynamic Environments

Dynamic environments include moving elements:

- Moving obstacles
- Changing lighting conditions
- Objects that respond to robot actions
- Other agents or robots in the environment

#### Creating Dynamic Objects

Dynamic objects can be created using Gazebo plugins:

```cpp
// Example plugin for a moving obstacle
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class MovingObstacle : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->physics = this->model->GetWorld()->Physics();

      // Connect to world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovingObstacle::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Move the object in a circular path
      static double angle = 0.0;
      double radius = 2.0;
      double x = radius * cos(angle);
      double y = radius * sin(angle);

      this->model->SetWorldPose(math::Pose(x, y, 0.5, 0, 0, angle));
      angle += 0.01;
    }

    private: physics::ModelPtr model;
    private: physics::PhysicsPtr physics;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(MovingObstacle)
}
```

## High-Fidelity Rendering in Unity

Unity provides high-fidelity rendering capabilities that complement Gazebo's physics simulation. This combination allows for:

- **Visual realism**: Photorealistic rendering of environments
- **Advanced lighting**: Complex lighting scenarios and shadows
- **Material properties**: Realistic surface appearances
- **Post-processing effects**: Depth of field, bloom, color grading

### Unity-Gazebo Integration Approaches

There are several approaches to integrate Unity with Gazebo:

1. **Direct Integration**: Using ROS# or similar bridges to connect Unity and ROS/Gazebo
2. **Data Exchange**: Exporting scene data from Gazebo and importing into Unity
3. **Synchronized Simulation**: Running both engines in parallel with synchronized states

#### Unity-ROS Bridge

The Unity-ROS bridge enables real-time communication:

```csharp
// Example Unity C# script for ROS communication
using UnityEngine;
using RosSharp;

public class RobotController : MonoBehaviour
{
    public string robotName = "my_robot";
    public float linearVelocity = 1.0f;
    public float angularVelocity = 1.0f;

    private RosSocket rosSocket;
    private TwistPublisher twistPublisher;

    void Start()
    {
        // Connect to ROS
        rosSocket = new RosSocket(new RosSharp.Protocols.WebSocketNetProtocol("ws://localhost:9090"));
        twistPublisher = GetComponent<TwistPublisher>();
        twistPublisher.Initialize(rosSocket, "/cmd_vel");
    }

    void Update()
    {
        // Send velocity commands to robot
        if (Input.GetKey(KeyCode.W))
        {
            twistPublisher.Publish(linearVelocity, 0, 0, 0, 0, angularVelocity);
        }
    }

    void OnDestroy()
    {
        rosSocket.Close();
    }
}
```

### Environment Export from Gazebo to Unity

To export environments from Gazebo to Unity:

1. Extract geometry and material information from SDF files
2. Convert coordinate systems (Gazebo uses ENU, Unity uses different conventions)
3. Apply appropriate scaling factors
4. Import textures and materials into Unity

#### Unity Scene Setup

Example Unity scene structure for robot simulation:

```
Scene Root
├── Environment
│   ├── Ground Plane
│   ├── Walls
│   ├── Obstacles
│   └── Furniture
├── Robot
│   ├── Robot Base
│   ├── Sensors (Visual representations)
│   └── Controllers
├── Cameras
│   ├── Main Camera
│   ├── Sensor Cameras
│   └── Overhead Camera
└── Lighting
    ├── Directional Light (Sun)
    ├── Point Lights
    └── Reflection Probes
```

## Human-Robot Interaction Concepts

### Types of Interaction

Human-robot interaction in simulation includes:

- **Visual interaction**: How humans perceive robot behavior
- **Communication**: Robot responses to human commands or presence
- **Collaboration**: Humans and robots working together
- **Safety**: Ensuring safe interaction between humans and robots

### Interaction Scenarios

Common interaction scenarios include:

1. **Assistive robotics**: Robots helping humans with daily tasks
2. **Collaborative robotics**: Humans and robots working together in industrial settings
3. **Social robotics**: Robots designed for social interaction
4. **Teleoperation**: Humans remotely controlling robots

### Designing for Human-Robot Interaction

When designing HRI scenarios in simulation:

1. **Consider human factors**: Response times, cognitive load, physical limitations
2. **Design intuitive interfaces**: Clear feedback, understandable robot behaviors
3. **Plan for safety**: Emergency stops, collision avoidance, safe distances
4. **Test various scenarios**: Different human operators, environmental conditions

## Practical Exercise: Environment Building

Let's build a custom environment in Gazebo:

1. Create a simple room with obstacles
2. Add interactive elements
3. Test human-robot interaction scenarios

```bash
# Build custom environments in Gazebo
ros2 launch digital_twin_examples environment_builder.launch.py
```

### Environment Building Best Practices

When building simulation environments:

- **Start simple**: Begin with basic shapes and gradually add complexity
- **Use realistic materials**: Apply appropriate textures and physical properties
- **Consider performance**: Balance visual quality with simulation speed
- **Validate against real environments**: Ensure simulation matches real-world conditions

## Exercises for Environment Building and Interaction

### Exercise 1: Basic Environment Creation
Create a simple indoor environment in Gazebo with:
- A rectangular room (5m x 4m)
- Two obstacles (chairs or tables)
- Proper lighting configuration
- A humanoid robot placed in the center

### Exercise 2: Dynamic Environment
Create a dynamic environment with:
- A moving obstacle that follows a predefined path
- A door that can be opened and closed by the robot
- Interactive objects that respond to robot contact

### Exercise 3: Human-Robot Interaction Scenario
Design and implement a simple HRI scenario:
- A robot assisting a human in a room
- The robot should navigate to objects requested by the human
- Implement safety behaviors when the human gets too close

### Exercise 4: Unity Integration
Export your Gazebo environment to Unity and:
- Apply high-quality materials and lighting
- Implement a first-person camera for human perspective
- Compare the visual quality between Gazebo and Unity representations

## Unity Integration

For high-fidelity rendering in Unity:

1. Export environment data from Gazebo
2. Import into Unity scene
3. Apply high-quality materials and lighting
4. Test rendering consistency with Gazebo

### Comparison of Gazebo vs Unity Rendering Capabilities

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Physics Simulation** | Excellent real-time physics | Limited physics (Unity Physics) |
| **Visual Quality** | Good for simulation | Excellent (photorealistic) |
| **Lighting** | Basic lighting models | Advanced lighting and shadows |
| **Materials** | Simple material properties | Complex shaders and materials |
| **Performance** | Optimized for real-time simulation | High-quality rendering (may impact performance) |
| **Integration** | Native ROS integration | Requires ROS bridge |
| **Realism** | Functional simulation | Photorealistic simulation |

## Summary

In this chapter, you learned about building robot environments in Gazebo and experiencing high-fidelity rendering in Unity. You now understand how to create realistic environments for testing and how human-robot interaction concepts apply in simulation. The next chapter will focus on sensor simulation and data flow into ROS 2 systems.