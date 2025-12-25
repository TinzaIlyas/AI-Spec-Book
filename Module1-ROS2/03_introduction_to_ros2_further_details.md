---
sidebar_position: 1
title: "Introduction to ROS 2"
---

# Module 1: ROS 2 - The Foundation
## Chapter 1: Introduction to ROS 2

### Overview
ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Concepts of ROS 2
ROS 2 provides a framework for developing robot applications with several core concepts:

**Nodes**: Processes that perform computation. Nodes are the fundamental building blocks of ROS 2 applications.

**Topics**: Named buses over which nodes exchange messages. Topics enable asynchronous message passing between nodes.

**Services**: Synchronous request/response communication between nodes. Services provide a way for nodes to send requests and receive responses.

**Actions**: Long-running tasks with feedback and goal management. Actions are useful for tasks that take time to complete.

### Why ROS 2 Over ROS 1?
ROS 2 addresses several limitations of ROS 1:

1. **Real-time support**: ROS 2 provides real-time capabilities for time-critical applications.
2. **Multi-robot systems**: Better support for multiple robots working together.
3. **Security**: Built-in security features for safe robot operation.
4. **Cross-platform compatibility**: Improved support for different operating systems.
5. **Quality of Service (QoS)**: Configurable communication policies for different requirements.

### ROS 2 Architecture
ROS 2 uses a DDS (Data Distribution Service) implementation for communication between nodes. This provides:

- **Discovery**: Nodes automatically discover each other on the network
- **Communication**: Reliable message passing between nodes
- **Quality of Service**: Configurable policies for message delivery

### Setting Up ROS 2
Before diving into ROS 2 development, you need to set up your environment:

1. Install ROS 2 distribution (Humble Hawksbill or Jazzy Jalisco)
2. Source the ROS 2 setup script
3. Create a workspace for your projects

```bash
# Install ROS 2
sudo apt install ros-humble-desktop

# Source the setup script
source /opt/ros/humble/setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Exercises and Questions

1. What is the main difference between ROS 1 and ROS 2?
2. Explain the purpose of nodes, topics, and services in ROS 2.
3. Why is real-time support important in robotics applications?
4. What is DDS and why does ROS 2 use it?
5. Describe the basic steps to set up a ROS 2 development environment.
6. What are Quality of Service (QoS) policies in ROS 2?
7. How does ROS 2 handle multi-robot systems differently from ROS 1?
8. What security features does ROS 2 provide?
9. Name three popular ROS 2 distributions.
10. Explain the purpose of the `colcon build` command in ROS 2.