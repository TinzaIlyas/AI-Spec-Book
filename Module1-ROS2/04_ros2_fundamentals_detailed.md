---
sidebar_position: 2
title: "ROS 2 Fundamentals"
---

# ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the core concepts of ROS 2 and its architecture
- Create and run ROS 2 nodes using Python (rclpy)
- Implement communication between nodes using Topics, Services, and Actions
- Define robot models using URDF (Unified Robot Description Format)
- Build complete ROS 2 packages with custom messages and services
- Debug and troubleshoot ROS 2 systems effectively

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and configurations.

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide additional features such as:
- Improved real-time support
- Better security and authentication
- Multi-robot systems support
- Cross-platform compatibility (Linux, Windows, macOS)
- Professional-grade quality and testing

### Key Concepts in ROS 2

1. **Nodes**: Processes that perform computation. ROS 2 is designed to be modular with nodes that perform different functions.

2. **Topics**: Named buses over which nodes exchange messages. Topics implement a publish/subscribe communication pattern.

3. **Services**: Synchronous communication pattern where a node sends a request and waits for a response.

4. **Actions**: Asynchronous communication pattern for long-running tasks with feedback and goal management.

5. **Packages**: Organizational units that contain code, data, and configuration files.

6. **Workspaces**: Directories where you modify and build ROS 2 packages.

## Setting Up ROS 2 Environment

Before diving into ROS 2 programming, you need to set up your environment. We'll use ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version.

### Installation Prerequisites

For Ubuntu 22.04 (recommended):
```bash
# Update package lists
sudo apt update

# Install required packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions
```

For other platforms, refer to the official ROS 2 installation guide.

### Environment Setup

Add the following line to your `~/.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
```

Then source your environment:
```bash
source ~/.bashrc
```

## ROS 2 Nodes and rclpy

### Understanding Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes can publish messages to topics, subscribe to topics to receive messages, provide services, call services, and more.

### Creating Your First ROS 2 Node

Let's create a simple ROS 2 package and node:

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a new package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_bringup --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

Now let's create a simple publisher node. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_publisher.py`:

```python
#!/usr/bin/env python3
"""
Simple publisher node that publishes messages to a topic.
This node demonstrates basic ROS 2 publisher functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A simple publisher node that publishes messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')

        # Create a publisher that publishes String messages to the 'chatter' topic
        # with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter to track the number of messages published
        self.i = 0

        # Log that the publisher has started
        self.get_logger().info('Simple Publisher has started')

    def timer_callback(self):
        """
        Callback method that is called by the timer.
        Creates and publishes a message to the 'chatter' topic.
        """
        # Create a String message
        msg = String()

        # Set the message data
        msg.data = f'Hello ROS 2 World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message for debugging
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SimplePublisher node
    simple_publisher = SimplePublisher()

    # Spin the node so the callback function is called
    rclpy.spin(simple_publisher)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    simple_publisher.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now let's create a corresponding subscriber node. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Simple subscriber node that subscribes to messages from a topic.
This node demonstrates basic ROS 2 subscriber functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A simple subscriber node that subscribes to messages from a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')

        # Create a subscription that subscribes to String messages from the 'chatter' topic
        # with a queue size of 10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Don't take ownership of the message
        self.subscription  # prevent unused variable warning

        # Log that the subscriber has started
        self.get_logger().info('Simple Subscriber has started')

    def listener_callback(self, msg):
        """
        Callback method that is called when a message is received.
        Prints the received message to the console.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SimpleSubscriber node
    simple_subscriber = SimpleSubscriber()

    # Spin the node so the callback function is called
    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    simple_subscriber.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now let's make these files executable and create the package setup:

```bash
chmod +x ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_publisher.py
chmod +x ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/simple_subscriber.py
```

Update the `setup.py` file in `~/ros2_ws/src/my_robot_bringup/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple ROS 2 publisher and subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_bringup.simple_publisher:main',
            'simple_subscriber = my_robot_bringup.simple_subscriber:main',
        ],
    },
)
```

Now build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

To run the nodes:
```bash
# Terminal 1: Run the publisher
ros2 run my_robot_bringup simple_publisher

# Terminal 2: Run the subscriber (in a new terminal, source the setup file first)
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_bringup simple_subscriber
```

## Topics - Publisher/Subscriber Communication

### Understanding Topics

Topics in ROS 2 implement a publish/subscribe communication pattern. Publishers send messages to a topic, and subscribers receive messages from a topic. Multiple publishers and subscribers can exist for the same topic.

### Advanced Topic Example: Sensor Data Publisher

Let's create a more complex example that simulates sensor data publishing. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/sensor_publisher.py`:

```python
#!/usr/bin/env python3
"""
Sensor data publisher node that publishes sensor readings.
This node demonstrates more complex message types and data publishing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math
import random


class SensorPublisher(Node):
    """
    A sensor publisher node that publishes simulated sensor data.
    """

    def __init__(self):
        # Initialize the node with the name 'sensor_publisher'
        super().__init__('sensor_publisher')

        # Create publishers for different sensor topics
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.point_pub = self.create_publisher(PointStamped, 'robot_position', 10)

        # Create a timer that calls the publish_sensor_data method every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

        # Initialize position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Log that the sensor publisher has started
        self.get_logger().info('Sensor Publisher has started')

    def publish_sensor_data(self):
        """
        Publish simulated sensor data including laser scan and position.
        """
        # Publish laser scan data
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'

        # Set laser scan parameters
        laser_msg.angle_min = -math.pi / 2  # -90 degrees
        laser_msg.angle_max = math.pi / 2   # 90 degrees
        laser_msg.angle_increment = math.pi / 180  # 1 degree increment
        laser_msg.time_increment = 0.0
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0

        # Generate simulated laser ranges (360 degrees)
        num_readings = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1
        laser_msg.ranges = []

        for i in range(num_readings):
            # Simulate some obstacles at random distances
            distance = 2.0 + random.uniform(-0.5, 0.5)  # Base distance of 2m with some noise
            # Occasionally add "obstacles" (shorter distances)
            if random.random() < 0.1:  # 10% chance of obstacle
                distance = random.uniform(0.5, 1.5)
            laser_msg.ranges.append(distance)

        # Publish the laser scan
        self.laser_pub.publish(laser_msg)

        # Publish robot position
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = self.x
        point_msg.point.y = self.y
        point_msg.point.z = 0.0

        # Update position (simulate robot movement)
        self.x += 0.01 * math.cos(self.theta)
        self.y += 0.01 * math.sin(self.theta)
        self.theta += random.uniform(-0.01, 0.01)  # Random rotation

        # Publish the position
        self.point_pub.publish(point_msg)

        # Log the publication
        self.get_logger().info(f'Published sensor data - X: {self.x:.2f}, Y: {self.y:.2f}')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SensorPublisher node
    sensor_publisher = SensorPublisher()

    # Spin the node so the callback function is called
    rclpy.spin(sensor_publisher)

    # Destroy the node explicitly
    sensor_publisher.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create a corresponding sensor subscriber: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/sensor_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Sensor data subscriber node that subscribes to sensor data.
This node demonstrates more complex message type subscription.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped


class SensorSubscriber(Node):
    """
    A sensor subscriber node that subscribes to sensor data.
    """

    def __init__(self):
        # Initialize the node with the name 'sensor_subscriber'
        super().__init__('sensor_subscriber')

        # Create subscriptions for different sensor topics
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        self.position_subscription = self.create_subscription(
            PointStamped,
            'robot_position',
            self.position_callback,
            10)

        # Log that the sensor subscriber has started
        self.get_logger().info('Sensor Subscriber has started')

    def laser_callback(self, msg):
        """
        Callback method for laser scan messages.
        """
        # Log basic information about the laser scan
        self.get_logger().info(
            f'Laser scan received - Min range: {min(msg.ranges):.2f}, '
            f'Max range: {max(msg.ranges):.2f}, '
            f'Number of readings: {len(msg.ranges)}'
        )

        # Find the closest obstacle
        min_range = min(msg.ranges)
        if min_range < 0.5:  # Less than 0.5m is considered close
            self.get_logger().warn('Obstacle detected very close!')
        elif min_range < 1.0:
            self.get_logger().info('Obstacle detected')

    def position_callback(self, msg):
        """
        Callback method for position messages.
        """
        # Log the robot's position
        self.get_logger().info(
            f'Robot position - X: {msg.point.x:.2f}, Y: {msg.point.y:.2f}'
        )


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SensorSubscriber node
    sensor_subscriber = SensorSubscriber()

    # Spin the node so the callback function is called
    rclpy.spin(sensor_subscriber)

    # Destroy the node explicitly
    sensor_subscriber.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Services - Request/Response Communication

### Understanding Services

Services in ROS 2 implement a request/response communication pattern. A client sends a request to a service server, which processes the request and returns a response.

### Creating a Custom Service

First, let's create a custom service definition. Create the directory structure and service file:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/srv
```

Create the service definition file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/srv/MoveRobot.srv`:

```
# Request
float64 x
float64 y
float64 theta
---
# Response
bool success
string message
```

Update the `package.xml` file to include the service:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_bringup</name>
  <version>0.0.0</version>
  <description>Simple ROS 2 publisher and subscriber example</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>message_runtime</depend>
  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Update the `setup.py` file to include the service:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple ROS 2 publisher and subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_bringup.simple_publisher:main',
            'simple_subscriber = my_robot_bringup.simple_subscriber:main',
            'sensor_publisher = my_robot_bringup.sensor_publisher:main',
            'sensor_subscriber = my_robot_bringup.sensor_subscriber:main',
            'robot_service_server = my_robot_bringup.robot_service_server:main',
            'robot_service_client = my_robot_bringup.robot_service_client:main',
        ],
    },
)
```

Now create the service server: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/robot_service_server.py`:

```python
#!/usr/bin/env python3
"""
Robot service server that provides robot movement capabilities.
This node demonstrates ROS 2 service implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from my_robot_bringup.srv import MoveRobot  # Import the custom service


class RobotServiceServer(Node):
    """
    A service server node that provides robot movement capabilities.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_service_server'
        super().__init__('robot_service_server')

        # Create a service server for the MoveRobot service
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)

        # Create a publisher for robot position updates
        self.position_pub = self.create_publisher(PointStamped, 'robot_position', 10)

        # Initialize robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Log that the service server has started
        self.get_logger().info('Robot Service Server has started')

    def move_robot_callback(self, request, response):
        """
        Callback method for the move_robot service.
        Processes the movement request and returns a response.
        """
        # Log the incoming request
        self.get_logger().info(
            f'Received move request - X: {request.x}, Y: {request.y}, Theta: {request.theta}'
        )

        # Simulate movement (in a real robot, this would control actual motors)
        self.current_x = request.x
        self.current_y = request.y
        self.current_theta = request.theta

        # Publish the new position
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'map'
        position_msg.point.x = self.current_x
        position_msg.point.y = self.current_y
        position_msg.point.z = 0.0

        self.position_pub.publish(position_msg)

        # Set the response
        response.success = True
        response.message = f'Moved robot to position ({request.x}, {request.y}, {request.theta})'

        # Log the response
        self.get_logger().info(f'Service response: {response.message}')

        return response


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RobotServiceServer node
    robot_service_server = RobotServiceServer()

    # Spin the node so the callback function is called
    rclpy.spin(robot_service_server)

    # Destroy the node explicitly
    robot_service_server.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create the service client: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/robot_service_client.py`:

```python
#!/usr/bin/env python3
"""
Robot service client that calls the robot movement service.
This node demonstrates ROS 2 service client implementation.
"""

import sys
import rclpy
from rclpy.node import Node
from my_robot_bringup.srv import MoveRobot  # Import the custom service


class RobotServiceClient(Node):
    """
    A service client node that calls the robot movement service.
    """

    def __init__(self):
        # Initialize the node with the name 'robot_service_client'
        super().__init__('robot_service_client')

        # Create a client for the MoveRobot service
        self.cli = self.create_client(MoveRobot, 'move_robot')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request
        self.request = MoveRobot.Request()

        # Log that the service client has started
        self.get_logger().info('Robot Service Client has started')

    def send_request(self, x, y, theta):
        """
        Send a movement request to the service server.
        """
        # Set the request parameters
        self.request.x = x
        self.request.y = y
        self.request.theta = theta

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)

        # Spin until the future is complete
        rclpy.spin_until_future_complete(self, self.future)

        # Return the response
        return self.future.result()


def main(args=None):
    """
    Main function that initializes the node and sends service requests.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RobotServiceClient node
    robot_service_client = RobotServiceClient()

    # Check if command line arguments are provided
    if len(sys.argv) != 4:
        print('Usage: ros2 run my_robot_bringup robot_service_client <x> <y> <theta>')
        print('Example: ros2 run my_robot_bringup robot_service_client 1.0 2.0 0.5')
        return

    # Parse command line arguments
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
    except ValueError:
        print('Error: Please provide numeric values for x, y, and theta')
        return

    # Send the request
    response = robot_service_client.send_request(x, y, theta)

    # Print the response
    if response:
        print(f'Response: {response.message}')
        print(f'Success: {response.success}')
    else:
        print('Service call failed')

    # Destroy the node explicitly
    robot_service_client.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions - Long-Running Tasks with Feedback

### Understanding Actions

Actions in ROS 2 are designed for long-running tasks that provide feedback during execution. They support goal setting, cancellation, and continuous feedback about the task's progress.

### Creating a Custom Action

First, let's create an action definition. Create the directory structure:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/action
```

Create the action definition file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/action/Navigation.action`:

```
# Goal: Target position to navigate to
float64 x
float64 y
float64 theta
---
# Result: Final status of the navigation
bool success
string message
float64 distance_traveled
---
# Feedback: Current progress during navigation
float64 distance_to_goal
float64 elapsed_time
string status
```

Update the setup.py to include the action:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple ROS 2 publisher and subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_bringup.simple_publisher:main',
            'simple_subscriber = my_robot_bringup.simple_subscriber:main',
            'sensor_publisher = my_robot_bringup.sensor_publisher:main',
            'sensor_subscriber = my_robot_bringup.sensor_subscriber:main',
            'robot_service_server = my_robot_bringup.robot_service_server:main',
            'robot_service_client = my_robot_bringup.robot_service_client:main',
            'navigation_action_server = my_robot_bringup.navigation_action_server:main',
            'navigation_action_client = my_robot_bringup.navigation_action_client:main',
        ],
    },
)
```

Now create the action server: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/navigation_action_server.py`:

```python
#!/usr/bin/env python3
"""
Navigation action server that provides robot navigation capabilities.
This node demonstrates ROS 2 action implementation.
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from my_robot_bringup.action import Navigation  # Import the custom action
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class NavigationActionServer(Node):
    """
    An action server node that provides robot navigation capabilities.
    """

    def __init__(self):
        # Initialize the node with the name 'navigation_action_server'
        super().__init__('navigation_action_server')

        # Create an action server for the Navigation action
        self._action_server = ActionServer(
            self,
            Navigation,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscription for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Initialize robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Log that the action server has started
        self.get_logger().info('Navigation Action Server has started')

    def goal_callback(self, goal_request):
        """
        Called when a new goal is received.
        """
        # Accept all goals
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when a goal is cancelled.
        """
        self.get_logger().info('Received request to cancel navigation goal')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        """
        Callback for odometry messages to track robot position.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract orientation from quaternion
        quat = msg.pose.pose.orientation
        self.current_theta = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

    def execute_callback(self, goal_handle):
        """
        Execute the navigation goal.
        """
        self.get_logger().info('Executing navigation goal...')

        # Get goal parameters
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        target_theta = goal_handle.request.theta

        # Initialize feedback and result
        feedback_msg = Navigation.Feedback()
        result = Navigation.Result()

        # Calculate initial distance to goal
        initial_distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        total_distance_traveled = 0.0
        start_time = time.time()

        # Navigation loop
        while rclpy.ok():
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Navigation goal canceled'
                result.distance_traveled = total_distance_traveled
                self.get_logger().info('Navigation goal canceled')
                return result

            # Calculate current distance to goal
            current_distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

            # Check if we've reached the goal (within tolerance)
            if current_distance < 0.1:  # 10cm tolerance
                # Stop the robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                goal_handle.succeed()
                result.success = True
                result.message = 'Navigation completed successfully'
                result.distance_traveled = total_distance_traveled
                self.get_logger().info('Navigation completed successfully')
                return result

            # Calculate required velocity to move toward goal
            cmd_vel = Twist()
            
            # Proportional controller for linear velocity
            linear_kp = 0.5
            cmd_vel.linear.x = min(linear_kp * current_distance, 0.5)  # Max 0.5 m/s
            
            # Proportional controller for angular velocity
            angular_kp = 2.0
            target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
            angle_error = target_angle - self.current_theta
            
            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
                
            cmd_vel.angular.z = angular_kp * angle_error

            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)

            # Update distance traveled
            total_distance_traveled = initial_distance - current_distance

            # Publish feedback
            feedback_msg.distance_to_goal = current_distance
            feedback_msg.elapsed_time = time.time() - start_time
            feedback_msg.status = f'Navigating - Distance to goal: {current_distance:.2f}m'
            goal_handle.publish_feedback(feedback_msg)

            # Log progress periodically
            if int(feedback_msg.elapsed_time) % 5 == 0:  # Log every 5 seconds
                self.get_logger().info(f'Navigation progress: {feedback_msg.status}')

            # Sleep to control loop rate
            time.sleep(0.1)

        # If we exit the loop without completing, return failure
        goal_handle.abort()
        result.success = False
        result.message = 'Navigation aborted due to internal error'
        result.distance_traveled = total_distance_traveled
        self.get_logger().info('Navigation aborted')
        return result


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the NavigationActionServer node
    navigation_action_server = NavigationActionServer()

    # Spin the node so the callback functions are called
    rclpy.spin(navigation_action_server)

    # Destroy the node explicitly
    navigation_action_server.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create the action client: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/navigation_action_client.py`:

```python
#!/usr/bin/env python3
"""
Navigation action client that calls the robot navigation action.
This node demonstrates ROS 2 action client implementation.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_bringup.action import Navigation  # Import the custom action


class NavigationActionClient(Node):
    """
    An action client node that calls the robot navigation action.
    """

    def __init__(self):
        # Initialize the node with the name 'navigation_action_client'
        super().__init__('navigation_action_client')

        # Create an action client for the Navigation action
        self._action_client = ActionClient(
            self,
            Navigation,
            'navigate_to_pose')

        # Log that the action client has started
        self.get_logger().info('Navigation Action Client has started')

    def send_goal(self, x, y, theta):
        """
        Send a navigation goal to the action server.
        """
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = Navigation.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add a callback for when the goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for when the goal response is received.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result future
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for feedback during navigation.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.status}')

    def get_result_callback(self, future):
        """
        Callback for when the result is received.
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        self.get_logger().info(f'Distance traveled: {result.distance_traveled:.2f}m')
        rclpy.shutdown()


def main(args=None):
    """
    Main function that initializes the node and sends action goals.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the NavigationActionClient node
    action_client = NavigationActionClient()

    # Check if command line arguments are provided
    if len(sys.argv) != 4:
        print('Usage: ros2 run my_robot_bringup navigation_action_client <x> <y> <theta>')
        print('Example: ros2 run my_robot_bringup navigation_action_client 1.0 2.0 0.5')
        return

    # Parse command line arguments
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
    except ValueError:
        print('Error: Please provide numeric values for x, y, and theta')
        return

    # Send the navigation goal
    action_client.send_goal(x, y, theta)

    # Spin to process callbacks
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## URDF - Robot Modeling

### Understanding URDF

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot's physical and visual properties, including links, joints, and materials.

### Creating a Simple Robot Model

Create a URDF file for a simple wheeled robot. Create the directory structure:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/urdf
```

Create the URDF file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/urdf/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Castor wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.175 0.0" rpy="1.57079632679 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.175 0.0" rpy="1.57079632679 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.2 0.0 -0.05"/>
  </joint>
</robot>
```

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2, including:
- Nodes and the rclpy library
- Topics for publish/subscribe communication
- Services for request/response communication
- Actions for long-running tasks with feedback
- URDF for robot modeling

These concepts form the foundation for developing complex robotic systems. The examples provided demonstrate practical implementations of each concept, which you can build upon for more sophisticated applications.