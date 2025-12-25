---
sidebar_position: 3
title: "Actions and Parameters"
---

# Module 1: ROS 2 - The Foundation
## Chapter 3: Actions and Parameters

### Overview
Actions and parameters are two additional communication patterns in ROS 2 that provide more sophisticated ways to interact with robotic systems. Actions are used for long-running tasks with feedback, while parameters allow runtime configuration of nodes.

### Actions - Goal-Based Communication
Actions provide a way to communicate with long-running tasks that require feedback and goal management. They're particularly useful for navigation, manipulation, and other complex robotic behaviors.

**Key Components of Actions:**
- **Goal**: Request sent to the action server to start a task
- **Result**: Final outcome of the action when completed
- **Feedback**: Periodic updates on the progress of the action

**Action Communication Flow:**
1. Client sends a goal to the action server
2. Server accepts or rejects the goal
3. Server sends periodic feedback during execution
4. Server sends final result when task completes
5. Client can cancel the goal during execution

**Example Action Definition (Fibonacci.action):**
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**Example Action Server:**
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Example Action Client:**
```python
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

### Parameters - Runtime Configuration
Parameters allow nodes to be configured at runtime without recompilation. They provide a way to change node behavior dynamically.

**Parameter Features:**
- Type-safe configuration values
- Declarative parameter definitions
- Parameter callbacks for dynamic behavior changes
- Parameter validation
- Parameter descriptors for constraints

**Example Parameter Usage:**
```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values and descriptors
        self.declare_parameter('robot_name', 'turtlebot4')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('sensor_range', 3.0)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.sensor_range = self.get_parameter('sensor_range').value
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info(f'Robot: {self.robot_name}, Max Vel: {self.max_velocity}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 2.0:
                    return SetParametersResult(successful=False, reason='Max velocity too high')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch Files - Managing Multiple Nodes
Launch files allow you to start multiple nodes with specific configurations simultaneously:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot4',
            description='Name of the robot'
        ),
        
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            parameters=[
                {'robot_name': LaunchConfiguration('robot_name')}
            ]
        ),
        
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener'
        )
    ])
```

### Exercises and Questions

1. What is the difference between actions and services in ROS 2?
2. When would you use an action instead of a topic or service?
3. Explain the three components of an action (Goal, Result, Feedback).
4. Write a simple action server that performs a navigation task.
5. Create an action client that monitors the progress of a navigation action.
6. How do parameters differ from topics in terms of configuration?
7. What are parameter descriptors and why are they useful?
8. Describe how to implement parameter validation in a ROS 2 node.
9. What are launch files and why are they important?
10. Create a launch file that starts multiple nodes with different parameters.