---
sidebar_position: 1
title: "Module 1: ROS 2 - The Foundation"
---

# Module 1: ROS 2 - The Foundation

## Learning Objectives

By the end of this module, you will be able to:
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
colcon build --packages-select
```

Wait, let me correct this. Here's the proper way to create a ROS 2 package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_bringup
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
        print('Error: Please provide valid floating point numbers for x, y, and theta')
        return
    
    # Send the service request
    print(f'Sending move request to ({x}, {y}, {theta})')
    response = robot_service_client.send_request(x, y, theta)
    
    # Print the response
    if response is not None:
        print(f'Response: Success={response.success}, Message="{response.message}"')
    else:
        print('Service call failed')
    
    # Destroy the node explicitly
    robot_service_client.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions - Long-Running Tasks

### Understanding Actions

Actions in ROS 2 are used for long-running tasks that provide feedback during execution and can be canceled. They are ideal for navigation, manipulation, and other tasks that take time to complete.

### Creating a Custom Action

First, let's create a custom action definition. Create the directory structure and action file:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/action
```

Create the action definition file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/action/Navigation.action`:

```
# Goal definition
float64 target_x
float64 target_y
float64 target_theta
---
# Result definition
bool success
string message
float64 final_x
float64 final_y
float64 final_theta
---
# Feedback definition
float64 current_x
float64 current_y
float64 current_theta
float64 distance_to_goal
string status
```

Update the `package.xml` file to include the action:

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
  <depend>action_msgs</depend>
  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Update the `setup.py` file to include the action:

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
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from my_robot_bringup.action import Navigation  # Import the custom action


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
        
        # Initialize robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Log that the action server has started
        self.get_logger().info('Navigation Action Server has started')
    
    def goal_callback(self, goal_request):
        """
        Accept or reject a client request to begin an action.
        """
        self.get_logger().info('Received navigation goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """
        Accept or reject a client request to cancel an action.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """
        Execute the navigation goal.
        """
        self.get_logger().info('Executing navigation goal...')
        
        # Get the target position from the goal
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        
        # Calculate the distance to the goal
        start_x = self.current_x
        start_y = self.current_y
        
        # Simulate navigation by gradually moving toward the target
        steps = 50  # Number of steps to simulate navigation
        for i in range(steps):
            if goal_handle.is_canceling():
                goal_handle.canceled()
                result = Navigation.Result()
                result.success = False
                result.message = 'Navigation was canceled'
                result.final_x = self.current_x
                result.final_y = self.current_y
                result.final_theta = self.current_theta
                return result
            
            # Calculate intermediate position (linear interpolation)
            progress = i / (steps - 1)
            self.current_x = start_x + (target_x - start_x) * progress
            self.current_y = start_y + (target_y - start_y) * progress
            self.current_theta = self.current_theta + (target_theta - self.current_theta) * progress
            
            # Calculate distance to goal
            distance_to_goal = ((target_x - self.current_x) ** 2 + 
                               (target_y - self.current_y) ** 2) ** 0.5
            
            # Publish feedback
            feedback_msg = Navigation.Feedback()
            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.current_theta = self.current_theta
            feedback_msg.distance_to_goal = distance_to_goal
            feedback_msg.status = f'Navigating... {progress*100:.1f}% complete'
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep to simulate navigation time
            time.sleep(0.1)
        
        # Check if goal was canceled during execution
        if goal_handle.is_canceling():
            goal_handle.canceled()
            result = Navigation.Result()
            result.success = False
            result.message = 'Navigation was canceled'
            result.final_x = self.current_x
            result.final_y = self.current_y
            result.final_theta = self.current_theta
        else:
            # Set the final position
            self.current_x = target_x
            self.current_y = target_y
            self.current_theta = target_theta
            
            # Succeed the goal
            goal_handle.succeed()
            result = Navigation.Result()
            result.success = True
            result.message = f'Navigation completed successfully to ({target_x}, {target_y}, {target_theta})'
            result.final_x = self.current_x
            result.final_y = self.current_y
            result.final_theta = self.current_theta
        
        self.get_logger().info(f'Navigation result: {result.message}')
        return result


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the NavigationActionServer node
    navigation_action_server = NavigationActionServer()
    
    # Spin the node so the action callbacks are called
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
Navigation action client that sends navigation goals to the action server.
This node demonstrates ROS 2 action client implementation.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_bringup.action import Navigation  # Import the custom action


class NavigationActionClient(Node):
    """
    An action client node that sends navigation goals to the action server.
    """
    
    def __init__(self):
        # Initialize the node with the name 'navigation_action_client'
        super().__init__('navigation_action_client')
        
        # Create an action client for the Navigation action
        self._action_client = ActionClient(self, Navigation, 'navigate_to_pose')
    
    def send_goal(self, target_x, target_y, target_theta):
        """
        Send a navigation goal to the action server.
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server to be available...')
        self._action_client.wait_for_server()
        
        # Create a goal message
        goal_msg = Navigation.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.target_theta = target_theta
        
        # Send the goal and get a future for the result
        self.get_logger().info(f'Sending navigation goal to ({target_x}, {target_y}, {target_theta})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        # Add a callback to handle the result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def goal_response_callback(self, future):
        """
        Handle the goal response from the action server.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server')
            return
        
        self.get_logger().info('Goal was accepted by the action server, waiting for result...')
        
        # Get the result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Handle the result from the action server.
        """
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}, Message="{result.message}"')
        self.get_logger().info(f'Final position: ({result.final_x}, {result.final_y}, {result.final_theta})')
    
    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server during execution.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status} - Distance to goal: {feedback.distance_to_goal:.2f}m')


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
        print('Example: ros2 run my_robot_bringup navigation_action_client 2.0 3.0 0.785')
        return
    
    # Parse command line arguments
    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_theta = float(sys.argv[3])
    except ValueError:
        print('Error: Please provide valid floating point numbers for x, y, and theta')
        return
    
    # Send the navigation goal
    future = action_client.send_goal(target_x, target_y, target_theta)
    
    # Spin to handle callbacks
    rclpy.spin(action_client)
    
    # Destroy the node explicitly
    action_client.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## URDF - Robot Modeling

### Understanding URDF

URDF (Unified Robot Description Format) is an XML format used to describe robots in ROS. It defines the robot's physical and visual properties, including links, joints, inertial properties, and visual representations.

### Creating a Simple Robot Model

Let's create a simple differential drive robot model. Create the URDF file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/urdf/simple_robot.urdf`:

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

  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>
  
  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>
  
  <gazebo reference="caster_wheel">
    <material>Gazebo/Grey</material>
  </gazebo>
</robot>
```

### URDF with ROS 2 Launch File

Create a launch file to spawn the robot in Gazebo. Create the directory and file:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/launch
```

Create the launch file `~/ros2_ws/src/my_robot_bringup/launch/robot_spawn.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_robot_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get URDF file path
    urdf_file_path = os.path.join(pkg_share, 'urdf/simple_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()
    
    # Create the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description}
        ]
    )
    
    # Create the joint state publisher node (for non-fixed joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Create the Gazebo spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
```

## Complete Working Example: Robot Controller

Let's create a complete example that brings together all the concepts learned. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/robot_controller.py`:

```python
#!/usr/bin/env python3
"""
Complete robot controller that demonstrates all ROS 2 concepts.
This node integrates topics, services, and actions for robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from my_robot_bringup.srv import MoveRobot
from my_robot_bringup.action import Navigation
import time


class RobotController(Node):
    """
    A complete robot controller that demonstrates all ROS 2 concepts.
    """
    
    def __init__(self):
        # Initialize the node with the name 'robot_controller'
        super().__init__('robot_controller')
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create a publisher for status updates
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # Create a client for the MoveRobot service
        self.move_service_client = self.create_client(MoveRobot, 'move_robot')
        
        # Create an action client for navigation
        self.nav_action_client = ActionClient(self, Navigation, 'navigate_to_pose')
        
        # Wait for services and actions to be available
        while not self.move_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move service not available, waiting again...')
        
        self.nav_action_client.wait_for_server()
        
        # Initialize robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Timer for periodic status updates
        self.timer = self.create_timer(1.0, self.publish_status)
        
        # Log that the controller has started
        self.get_logger().info('Robot Controller has started')
    
    def publish_status(self):
        """
        Publish the current robot status.
        """
        status_msg = String()
        status_msg.data = f'Robot at ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_theta:.2f})'
        self.status_pub.publish(status_msg)
    
    def move_robot_direct(self, linear_x, angular_z):
        """
        Send direct velocity commands to the robot.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f'Sent velocity command: linear_x={linear_x}, angular_z={angular_z}')
    
    def move_robot_service(self, x, y, theta):
        """
        Use the MoveRobot service to move the robot.
        """
        request = MoveRobot.Request()
        request.x = x
        request.y = y
        request.theta = theta
        
        future = self.move_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is not None:
            self.get_logger().info(f'Move service response: {response.message}')
            # Update internal state based on service response
            self.current_x = x
            self.current_y = y
            self.current_theta = theta
        else:
            self.get_logger().error('Move service call failed')
    
    def navigate_to_pose(self, target_x, target_y, target_theta):
        """
        Use the navigation action to move to a specific pose.
        """
        goal_msg = Navigation.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.target_theta = target_theta
        
        self.get_logger().info(f'Sending navigation goal to ({target_x}, {target_y}, {target_theta})')
        
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback)
        
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)
        
        return send_goal_future
    
    def navigation_goal_response_callback(self, future):
        """
        Handle the navigation goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal was rejected')
            return
        
        self.get_logger().info('Navigation goal accepted, waiting for result...')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """
        Handle the navigation result.
        """
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result.message}')
        
        # Update internal state based on result
        self.current_x = result.final_x
        self.current_y = result.final_y
        self.current_theta = result.final_theta
    
    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation feedback: {feedback.status} - '
            f'Distance: {feedback.distance_to_goal:.2f}m'
        )


def main(args=None):
    """
    Main function that demonstrates all ROS 2 concepts.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the RobotController node
    controller = RobotController()
    
    # Demonstrate direct velocity control
    controller.get_logger().info('Demonstrating direct velocity control...')
    controller.move_robot_direct(0.5, 0.0)  # Move forward
    time.sleep(2)
    controller.move_robot_direct(0.0, 0.5)  # Turn right
    time.sleep(2)
    controller.move_robot_direct(0.0, 0.0)  # Stop
    
    # Demonstrate service call
    controller.get_logger().info('Demonstrating service call...')
    controller.move_robot_service(1.0, 1.0, 0.0)
    time.sleep(2)
    
    # Demonstrate action call
    controller.get_logger().info('Demonstrating action call...')
    future = controller.navigate_to_pose(2.0, 2.0, 1.57)  # Navigate to (2,2) with 90-degree rotation
    
    # Spin until the action is complete
    rclpy.spin_until_future_complete(controller, future)
    
    # Clean shutdown
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update the `setup.py` file to include the new node:

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
        ('share/' + package_name + '/urdf', ['my_robot_bringup/urdf/simple_robot.urdf']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
        ('share/' + package_name + '/launch', ['launch/robot_spawn.launch.py']),
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
            'robot_controller = my_robot_bringup.robot_controller:main',
        ],
    },
)
```

Now let's build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

## Exercises and Mini Projects

### Exercise 1: Custom Message Types
Create a custom message type to represent a robot's battery status and implement a publisher and subscriber for this message.

**Solution:**

First, create the message definition. Create the directory and file:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/msg
```

Create the message file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/msg/BatteryStatus.msg`:

```
# Battery status message
float64 voltage
float64 current
float64 charge_percentage
bool charging
time last_update
```

Update the `package.xml` to include the message:

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
  <depend>action_msgs</depend>
  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Update the `setup.py` to include the message:

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
        ('share/' + package_name + '/urdf', ['my_robot_bringup/urdf/simple_robot.urdf']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
        ('share/' + package_name + '/msg', ['my_robot_bringup/msg/BatteryStatus.msg']),
        ('share/' + package_name + '/launch', ['launch/robot_spawn.launch.py']),
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
            'robot_controller = my_robot_bringup.robot_controller:main',
            'battery_publisher = my_robot_bringup.battery_publisher:main',
            'battery_subscriber = my_robot_bringup.battery_subscriber:main',
        ],
    },
)
```

Create the battery publisher: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/battery_publisher.py`:

```python
#!/usr/bin/env python3
"""
Battery status publisher node.
This node demonstrates custom message types.
"""

import rclpy
from rclpy.node import Node
from my_robot_bringup.msg import BatteryStatus  # Import custom message
import random


class BatteryPublisher(Node):
    """
    A publisher node that publishes battery status messages.
    """
    
    def __init__(self):
        # Initialize the node with the name 'battery_publisher'
        super().__init__('battery_publisher')
        
        # Create a publisher for the custom BatteryStatus message
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery_status', 10)
        
        # Create a timer that calls the publish_battery_status method every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_battery_status)
        
        # Initialize battery state
        self.battery_level = 100.0
        
        # Log that the publisher has started
        self.get_logger().info('Battery Publisher has started')
    
    def publish_battery_status(self):
        """
        Publish battery status message.
        """
        # Create a BatteryStatus message
        msg = BatteryStatus()
        
        # Set the message fields
        msg.voltage = 12.6 + random.uniform(-0.2, 0.2)  # Simulate voltage variation
        msg.current = random.uniform(0.5, 2.0)  # Simulate current draw
        msg.charge_percentage = self.battery_level
        msg.charging = False  # Simulate discharging
        
        # Update time field
        msg.last_update = self.get_clock().now().to_msg()
        
        # Decrease battery level over time
        self.battery_level = max(0.0, self.battery_level - 0.1)
        
        # If battery is low, reset to 100% (simulating recharge)
        if self.battery_level <= 0:
            self.battery_level = 100.0
            msg.charging = True
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the battery status
        self.get_logger().info(
            f'Battery Status - Voltage: {msg.voltage:.2f}V, '
            f'Level: {msg.charge_percentage:.1f}%, '
            f'Current: {msg.current:.2f}A, '
            f'Charging: {msg.charging}'
        )


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the BatteryPublisher node
    battery_publisher = BatteryPublisher()
    
    # Spin the node so the callback function is called
    rclpy.spin(battery_publisher)
    
    # Destroy the node explicitly
    battery_publisher.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create the battery subscriber: `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/battery_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Battery status subscriber node.
This node demonstrates custom message type subscription.
"""

import rclpy
from rclpy.node import Node
from my_robot_bringup.msg import BatteryStatus  # Import custom message


class BatterySubscriber(Node):
    """
    A subscriber node that subscribes to battery status messages.
    """
    
    def __init__(self):
        # Initialize the node with the name 'battery_subscriber'
        super().__init__('battery_subscriber')
        
        # Create a subscription for the custom BatteryStatus message
        self.subscription = self.create_subscription(
            BatteryStatus,
            'battery_status',
            self.battery_callback,
            10)
        
        # Don't take ownership of the message
        self.subscription  # prevent unused variable warning
        
        # Log that the subscriber has started
        self.get_logger().info('Battery Subscriber has started')
    
    def battery_callback(self, msg):
        """
        Callback method for battery status messages.
        """
        # Log the battery status
        if msg.charge_percentage < 20.0:
            self.get_logger().error(
                f'CRITICAL BATTERY LEVEL: {msg.charge_percentage:.1f}% - '
                f'Voltage: {msg.voltage:.2f}V'
            )
        elif msg.charge_percentage < 50.0:
            self.get_logger().warn(
                f'LOW BATTERY: {msg.charge_percentage:.1f}% - '
                f'Current: {msg.current:.2f}A'
            )
        else:
            self.get_logger().info(
                f'Battery OK: {msg.charge_percentage:.1f}% - '
                f'Voltage: {msg.voltage:.2f}V'
            )


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the BatterySubscriber node
    battery_subscriber = BatterySubscriber()
    
    # Spin the node so the callback function is called
    rclpy.spin(battery_subscriber)
    
    # Destroy the node explicitly
    battery_subscriber.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Mini Project: Robot Fleet Manager

Create a system that manages multiple robots with different capabilities. Each robot publishes its status, and a fleet manager coordinates their activities.

This project would involve creating multiple nodes that communicate with each other to coordinate robot activities, demonstrating all the concepts learned in this module.

## Summary

In this module, we've covered the foundational concepts of ROS 2:

1. **Nodes**: The basic building blocks of ROS 2 applications
2. **Topics**: Publish/subscribe communication for real-time data exchange
3. **Services**: Request/response communication for specific tasks
4. **Actions**: Long-running tasks with feedback and cancellation
5. **URDF**: Robot modeling for simulation and visualization
6. **Custom Messages/Services/Actions**: Extending ROS 2 for specific needs

These concepts form the foundation for all subsequent modules in this course. Understanding these fundamentals is crucial for building complex robotic systems that can perceive, reason, and act in the physical world.

In the next module, we'll explore simulation environments where we can test and validate our ROS 2 systems before deploying them on physical robots.