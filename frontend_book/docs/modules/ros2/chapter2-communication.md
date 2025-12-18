---
sidebar_position: 2
title: "Chapter 2: ROS 2 Communication"
---

# Chapter 2: ROS 2 Communication

## Nodes, Topics, Services, Actions

ROS 2 communication is built around several core concepts that enable distributed robot systems to work together effectively.

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node performs a specific task and communicates with other nodes through topics, services, and actions. In ROS 2, nodes are implemented as objects that can be grouped into processes (executables). Multiple nodes can exist within a single process, and a single node can exist in multiple processes.

### Topics
Topics enable asynchronous, many-to-many communication through a publish-subscribe model. Publishers send messages to topics, and subscribers receive messages from topics. This decouples the sender and receiver in time, space, and synchronization.

### Services
Services provide synchronous, request-response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response. This is similar to REST API calls.

### Actions
Actions are used for long-running tasks that require feedback and the ability to cancel. They combine the features of topics and services, providing goal requests, feedback during execution, and final results.

## Robot Coordination via Message Passing

ROS 2 uses Data Distribution Service (DDS) as the underlying communication middleware. DDS provides a publish-subscribe pattern that enables loose coupling between publishers and subscribers.

### Quality of Service (QoS) Settings
ROS 2 provides QoS settings that allow fine-tuning of communication behavior:
- Reliability: Best effort vs. reliable delivery
- Durability: Volatile vs. transient local
- History: Keep last N samples vs. keep all samples

## Bridging AI Agents to ROS using Client Libraries

ROS 2 client libraries (rcl) provide language-specific APIs for interacting with ROS 2. Common client libraries include:
- rclcpp for C++
- rclpy for Python
- rclnodejs for JavaScript
- rclrs for Rust

These libraries allow AI agents written in different languages to communicate with ROS 2 systems. For example, a Python-based AI agent can use rclpy to publish sensor data to topics that a C++-based controller subscribes to.

### Example: Python AI Agent Integration
```python
import rclpy
from std_msgs.msg import String

def ai_agent_publisher():
    rclpy.init()
    node = rclpy.create_node('ai_agent')
    publisher = node.create_publisher(String, 'ai_decisions', 10)

    # AI agent logic here
    msg = String()
    msg.data = 'Decision from AI agent'
    publisher.publish(msg)
```

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the differences between nodes, topics, services, and actions
- Implement basic communication patterns in ROS 2
- Bridge AI agents to ROS systems using client libraries

## Exercises

1. Create a simple publisher and subscriber pair in ROS 2.
2. Implement a service client and server in ROS 2.
3. Research and explain the Quality of Service (QoS) settings in ROS 2.