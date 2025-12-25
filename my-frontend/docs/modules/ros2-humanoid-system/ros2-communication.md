---
title: ROS 2 Communication Model
sidebar_label: Communication Model
---

# ROS 2 Communication Model

## Understanding Nodes

In ROS 2, a **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. Each node is designed to perform a specific task and communicate with other nodes through topics, services, and actions.

### Creating a Node

Nodes in ROS 2 are typically implemented as classes that inherit from `rclpy.Node` (Python) or `rclcpp::Node` (C++). Each node should have a unique name within the ROS graph.

### Node Lifecycle

Nodes in ROS 2 have a well-defined lifecycle:
- **Unconfigured**: Node created but not configured
- **Inactive**: Node configured but not active
- **Active**: Node is running and participating in communication
- **Finalized**: Node is shutting down

## Topics and Message Passing

**Topics** enable asynchronous, many-to-many communication between nodes. Messages are published to topics and can be received by any number of subscribers.

### Key Concepts:
- **Publishers** send messages to topics
- **Subscribers** receive messages from topics
- Communication is decoupled and asynchronous
- Topics support different message types defined in `.msg` files

### Example Use Cases:
- Sensor data broadcasting (e.g., camera images, laser scans)
- Robot state publishing (e.g., joint positions, odometry)
- Event notifications

## Services and Request-Response

**Services** provide synchronous, request-response communication between nodes. A service client sends a request and waits for a response from a service server.

### Key Concepts:
- **Service Server** provides a specific functionality
- **Service Client** requests that functionality
- Communication is synchronous and blocking
- Services use `.srv` files to define request/response types

### Example Use Cases:
- Robot calibration
- Map saving/loading
- Parameter configuration

## Actions for Long-Running Tasks

**Actions** are designed for long-running tasks that require:
- Goal request from client
- Continuous feedback during execution
- Result upon completion
- Ability to cancel/abort the task

### Key Concepts:
- **Action Server** executes the long-running task
- **Action Client** sends goals and receives feedback/results
- Actions use `.action` files to define goal/feedback/result types

### Example Use Cases:
- Navigation to a goal location
- Object manipulation sequences
- Trajectory execution

## Implementing an Agent Controller

A basic agent controller in ROS 2 would typically involve:

1. **Sensor Integration**: Subscribing to sensor topics to gather environmental information
2. **Decision Making**: Processing sensor data and making control decisions
3. **Actuator Commands**: Publishing commands to actuator topics
4. **State Management**: Maintaining and updating the agent's internal state
5. **Safety Mechanisms**: Implementing safety checks and emergency stops

### Example Architecture:
```
Sensor Data → Perception → Decision Making → Action Selection → Actuator Commands
    ↓            ↓              ↓                ↓               ↓
State Update ← Control Loop ← Planning ← Behavior Arb. ← Safety Checks
```

## Quality of Service (QoS) Considerations

When designing communication patterns for humanoid robots, consider:

- **Reliability**: Use reliable delivery for critical safety messages, best effort for sensor streams
- **Durability**: Transient local for configuration parameters, volatile for sensor data
- **History**: Keep last N samples for sensor buffers, keep all for critical events
- **Deadline**: Set appropriate deadlines for real-time requirements
- **Liveliness**: Monitor node health in safety-critical applications

The next chapter will cover how to structure your robot using URDF (Unified Robot Description Format) for both physical robots and simulation.