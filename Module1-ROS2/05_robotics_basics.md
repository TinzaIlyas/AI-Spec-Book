---
sidebar_position: 3
title: "Robotics Basics"
---

# Robotics Basics

## Overview

This chapter covers the fundamental concepts of robotics that form the foundation for understanding more advanced topics in humanoid robotics. We'll explore the basic components of robotic systems, kinematics, dynamics, and control theory.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the basic components of robotic systems
- Explain kinematics and its importance in robotics
- Describe different types of robot actuators and sensors
- Apply basic control theory concepts to robotic systems
- Design simple robot control algorithms

## Basic Components of Robotic Systems

A robotic system typically consists of several key components that work together to achieve desired tasks:

### 1. Mechanical Structure
The physical body of the robot, including:
- Links: Rigid bodies that form the structure
- Joints: Connections between links that allow relative motion
- End-effectors: Tools or grippers at the end of manipulator arms

### 2. Actuators
Devices that provide motion to the robot:
- Electric motors (DC, stepper, servo)
- Hydraulic actuators
- Pneumatic actuators
- Shape memory alloys (for specialized applications)

### 3. Sensors
Devices that provide information about the robot and its environment:
- Position sensors (encoders, potentiometers)
- Force/torque sensors
- Vision systems (cameras, LIDAR)
- Inertial measurement units (IMUs)
- Tactile sensors

### 4. Control System
The "brain" of the robot that processes sensor data and commands actuators:
- Microcontrollers
- Single-board computers
- Real-time operating systems
- Control algorithms

## Kinematics

Kinematics is the study of motion without considering the forces that cause it. In robotics, kinematics deals with the relationship between joint positions and the position and orientation of the robot's end-effector.

### Forward Kinematics
Forward kinematics calculates the position and orientation of the end-effector given the joint angles. For a simple 2-link planar manipulator:

```
x = l1 * cos(θ1) + l2 * cos(θ1 + θ2)
y = l1 * sin(θ1) + l2 * sin(θ1 + θ2)
```

Where:
- l1, l2 are link lengths
- θ1, θ2 are joint angles

### Inverse Kinematics
Inverse kinematics calculates the required joint angles to achieve a desired end-effector position. This is often more complex than forward kinematics and may have multiple solutions or no solution.

## Dynamics

Robot dynamics deals with the forces and torques required to produce motion. The dynamics of a robot are described by the equation:

```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
```

Where:
- M(q) is the mass matrix
- C(q,q̇) contains Coriolis and centrifugal terms
- G(q) represents gravitational forces
- τ represents applied torques
- q, q̇, q̈ are joint positions, velocities, and accelerations

## Control Theory in Robotics

Control theory provides the mathematical framework for designing robot controllers that achieve desired behaviors.

### Open-Loop vs Closed-Loop Control

**Open-loop control** sends commands to the robot without feedback:
- Simple to implement
- No correction for disturbances or errors
- Suitable for predictable environments

**Closed-loop control** uses sensor feedback to correct errors:
- More robust to disturbances
- Can handle uncertain environments
- More complex to design and implement

### PID Control

Proportional-Integral-Derivative (PID) control is widely used in robotics:

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

Where:
- u(t) is the control signal
- e(t) is the error (desired - actual)
- Kp, Ki, Kd are the proportional, integral, and derivative gains

### Implementing PID Control in ROS 2

Here's an example of implementing a PID controller in ROS 2:

```python
#!/usr/bin/env python3
"""
PID controller node for robot control.
This node demonstrates PID control implementation in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class PIDController(Node):
    """
    A PID controller node for robot control.
    """

    def __init__(self):
        # Initialize the node with the name 'pid_controller'
        super().__init__('pid_controller')

        # Create subscriptions
        self.setpoint_sub = self.create_subscription(
            Float64,
            'setpoint',
            self.setpoint_callback,
            10)

        self.feedback_sub = self.create_subscription(
            Float64,
            'feedback',
            self.feedback_callback,
            10)

        # Create publisher for control output
        self.control_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # PID parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain

        # PID state variables
        self.setpoint = 0.0
        self.feedback = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.derivative = 0.0
        self.output = 0.0

        # Time step for integration
        self.dt = 0.01  # 10ms

        # Create a timer for PID computation
        self.timer = self.create_timer(self.dt, self.pid_compute)

        # Log that the PID controller has started
        self.get_logger().info('PID Controller has started')

    def setpoint_callback(self, msg):
        """
        Callback for setpoint messages.
        """
        self.setpoint = msg.data

    def feedback_callback(self, msg):
        """
        Callback for feedback messages.
        """
        self.feedback = msg.data

    def pid_compute(self):
        """
        Compute PID output.
        """
        # Calculate error
        self.error = self.setpoint - self.feedback

        # Calculate integral (with anti-windup)
        self.integral += self.error * self.dt
        # Limit integral to prevent windup
        max_integral = 10.0
        self.integral = max(min(self.integral, max_integral), -max_integral)

        # Calculate derivative
        self.derivative = (self.error - self.previous_error) / self.dt

        # Calculate PID output
        self.output = (self.kp * self.error + 
                      self.ki * self.integral + 
                      self.kd * self.derivative)

        # Store current error for next derivative calculation
        self.previous_error = self.error

        # Publish control output as Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.output  # For velocity control
        self.control_pub.publish(twist_msg)

        # Log PID values
        self.get_logger().info(
            f'PID - Setpoint: {self.setpoint:.2f}, '
            f'Feedback: {self.feedback:.2f}, '
            f'Error: {self.error:.2f}, '
            f'Output: {self.output:.2f}')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the PIDController node
    pid_controller = PIDController()

    # Spin the node so the callback functions are called
    rclpy.spin(pid_controller)

    # Destroy the node explicitly
    pid_controller.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Types of Robot Control

### Joint Space Control
Controls individual joints independently. Each joint has its own controller that tracks desired joint positions.

### Operational Space Control
Controls the end-effector directly in Cartesian space. More intuitive for task-oriented control.

### Impedance Control
Controls the robot's interaction with the environment by specifying desired mechanical impedance properties.

## Sensors in Robotics

### Position Sensors
- Encoders: Measure joint angles
- Potentiometers: Measure linear or angular position
- Resolvers: High-precision angular position sensors

### Force/Torque Sensors
- Strain gauges: Measure forces and torques
- Load cells: Measure weight or force
- Six-axis force/torque sensors: Measure forces and torques in all directions

### Vision Sensors
- Cameras: Provide visual information
- LIDAR: Provide 3D distance measurements
- Time-of-flight sensors: Measure distances using light

### Inertial Sensors
- Accelerometers: Measure linear acceleration
- Gyroscopes: Measure angular velocity
- IMUs: Combine accelerometers and gyroscopes

## Actuators in Robotics

### Electric Motors
- DC motors: Simple and cost-effective
- Stepper motors: Precise position control
- Servo motors: Integrated control electronics

### Hydraulic and Pneumatic Actuators
- High power-to-weight ratio
- Precise force control
- More complex infrastructure requirements

## Safety Considerations

Safety is paramount in robotics, especially for humanoid robots that operate near humans:

### Physical Safety
- Emergency stops
- Collision detection and avoidance
- Speed and force limitations
- Safe failure modes

### Operational Safety
- Redundant sensors
- Fail-safe mechanisms
- Proper error handling
- Safe human-robot interaction protocols

## Practical Example: Simple Robot Control

Let's create a complete example that combines several concepts we've learned:

```python
#!/usr/bin/env python3
"""
Complete robot control example combining multiple concepts.
This node demonstrates integrated robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class RobotController(Node):
    """
    A complete robot controller demonstrating multiple concepts.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_controller'
        super().__init__('robot_controller')

        # Create subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Create publishers
        self.joint_cmd_pub = self.create_publisher(Float64, 'joint_command', 10)

        # Robot state
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}
        self.desired_twist = Twist()

        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_separation = 0.3  # meters (for differential drive)

        # Create a timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Log that the robot controller has started
        self.get_logger().info('Robot Controller has started')

    def joint_state_callback(self, msg):
        """
        Callback for joint state messages.
        """
        # Update current joint states
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.current_efforts[name] = msg.effort[i]

    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands.
        """
        self.desired_twist = msg

    def control_loop(self):
        """
        Main control loop.
        """
        # Convert desired linear/angular velocities to wheel velocities
        # for differential drive robot
        linear_vel = self.desired_twist.linear.x
        angular_vel = self.desired_twist.angular.z

        # Calculate wheel velocities
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2) / self.wheel_radius

        # For this example, we'll just command the left wheel
        # In a real implementation, you'd command both wheels
        cmd_msg = Float64()
        cmd_msg.data = left_wheel_vel
        self.joint_cmd_pub.publish(cmd_msg)

        # Log control information
        self.get_logger().info(
            f'Control - Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}, '
            f'Left Wheel: {left_wheel_vel:.2f}')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RobotController node
    robot_controller = RobotController()

    # Spin the node so the callback functions are called
    rclpy.spin(robot_controller)

    # Destroy the node explicitly
    robot_controller.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Summary

This chapter has introduced the fundamental concepts of robotics, including:
- Basic components of robotic systems
- Kinematics (forward and inverse)
- Dynamics of robotic systems
- Control theory and PID controllers
- Types of sensors and actuators
- Safety considerations in robotics

These concepts provide the foundation for understanding more advanced topics in humanoid robotics. The practical examples demonstrate how these concepts are implemented in real robotic systems using ROS 2.