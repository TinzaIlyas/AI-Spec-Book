---
sidebar_position: 1
title: "Appendix A: ROS 2 Commands and Tools Reference"
---

# Appendix A: ROS 2 Commands and Tools Reference

## Overview
This appendix provides a comprehensive reference for essential ROS 2 commands, tools, and utilities that are frequently used in robotics development. This serves as a quick reference guide for common operations.

### Essential ROS 2 Commands

#### Basic Commands
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check ROS 2 installation
ros2 --version

# Get help on ROS 2 commands
ros2 --help
```

#### Package Management
```bash
# Create a new package
ros2 pkg create --build-type ament_python my_package
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs my_cpp_package

# List all packages in workspace
colcon list

# Find a specific package
ros2 pkg list | grep package_name
```

#### Node Management
```bash
# List all running nodes
ros2 node list

# Get information about a specific node
ros2 node info /node_name

# Run a node
ros2 run package_name executable_name

# Run a node with parameters
ros2 run package_name executable_name --ros-args --param param_name:=param_value
```

#### Topic Communication
```bash
# List all topics
ros2 topic list

# Get information about a topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name
ros2 topic echo /topic_name --field field_name

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/String "data: 'Hello World'"

# Show topic statistics
ros2 topic hz /topic_name
ros2 topic bw /topic_name
```

#### Service Communication
```bash
# List all services
ros2 service list

# Get information about a service
ros2 service info /service_name

# Call a service
ros2 service call /service_name example_interfaces/AddTwoInts "{a: 1, b: 2}"

# Find service type
ros2 service type /service_name
```

#### Action Communication
```bash
# List all actions
ros2 action list

# Get information about an action
ros2 action info /action_name

# Send a goal to an action server
ros2 action send_goal /fibonacci example_interfaces/Fibonacci "{order: 5}"

# Find action type
ros2 action type /action_name
```

#### Parameter Management
```bash
# List parameters of a node
ros2 param list /node_name

# Get a parameter value
ros2 param get /node_name param_name

# Set a parameter value
ros2 param set /node_name param_name new_value

# Load parameters from a file
ros2 param load /node_name /path/to/params.yaml

# Dump parameters to a file
ros2 param dump /node_name
```

### Launch System Commands
```bash
# Run a launch file
ros2 launch package_name launch_file.py

# Run with arguments
ros2 launch package_name launch_file.py arg_name:=arg_value

# List launch files in a package
ros2 launch --list package_name
```

### Building and Development Tools
```bash
# Build the workspace
colcon build
colcon build --packages-select package_name
colcon build --symlink-install

# Build with specific options
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --ament-cmake-args --parallel-workers 4

# Run tests
colcon test
colcon test --packages-select package_name
colcon test-result --all

# Clean build artifacts
rm -rf build/ install/ log/
```

### Debugging and Monitoring Tools
```bash
# Visualize the ROS 2 graph
rqt_graph

# Plot numerical data
rqt_plot

# View log messages
rqt_console

# Monitor topics and services
rqt

# Record bag files
ros2 bag record -a
ros2 bag record /topic1 /topic2

# Play back bag files
ros2 bag play /path/to/bag_file

# Check network connectivity
ros2 doctor
```

### Common Message Types
```bash
# Standard message types
std_msgs/String
std_msgs/Int32
std_msgs/Float64
std_msgs/Bool

# Geometry messages
geometry_msgs/Pose
geometry_msgs/Twist
geometry_msgs/Point
geometry_msgs/Quaternion

# Sensor messages
sensor_msgs/Image
sensor_msgs/LaserScan
sensor_msgs/JointState
sensor_msgs/BatteryState

# Navigation messages
nav_msgs/Odometry
nav_msgs/Path
nav_msgs/MapMetaData

# Custom message types
example_interfaces/AddTwoInts
```

### Quality of Service (QoS) Settings
```bash
# Common QoS profiles
sensor_data: Default for sensor data
services: Default for services
parameters: Default for parameters
system_default: Default for system use

# Example with custom QoS
ros2 topic echo /topic_name --qos-profile sensor_data
```

### Working with Multiple Machines
```bash
# Set ROS domain ID
export ROS_DOMAIN_ID=1

# Set ROS local hostname
export ROS_LOCALHOST_ONLY=0

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

### Environment Setup Scripts
```bash
# ~/.bashrc additions for ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Custom aliases
alias cb='colcon build'
alias cs='cd ~/ros2_ws && source install/setup.bash'
alias rt='ros2 topic'
alias rn='ros2 node'
alias rs='ros2 service'
```

### Common Error Solutions
```bash
# Permission denied errors
sudo chmod +x /path/to/executable

# Package not found errors
source install/setup.bash
export AMENT_PREFIX_PATH=~/ros2_ws/install:$AMENT_PREFIX_PATH

# Topic not found errors
ros2 run package_name node_name
ros2 topic list  # Verify topic exists
```

### Performance Monitoring
```bash
# Monitor node performance
ros2 run top top_node

# Monitor topic rates
ros2 topic hz /topic_name

# Monitor system resources
htop
iotop
nethogs
```

### Troubleshooting Commands
```bash
# Check ROS 2 environment
printenv | grep ROS

# Verify ROS 2 daemon
ros2 daemon status
ros2 daemon start
ros2 daemon stop

# Check for port conflicts
netstat -tulpn | grep 11711
```

### Exercises and Questions

1. How do you create a new ROS 2 package?
2. What command shows all running ROS 2 nodes?
3. How do you publish a message to a topic?
4. What is the command to call a service?
5. How do you set a parameter value at runtime?
6. How do you record and play back bag files?
7. What does the `colcon build --symlink-install` command do?
8. How do you run a launch file with arguments?
9. What is the purpose of Quality of Service settings?
10. How do you troubleshoot ROS 2 network connectivity issues?