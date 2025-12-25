---
sidebar_position: 2
title: "Nodes, Topics, and Services"
---

# Module 1: ROS 2 - The Foundation
## Chapter 2: Nodes, Topics, and Services

### Overview
Understanding the fundamental communication patterns in ROS 2 is crucial for building effective robotic applications. Nodes, topics, and services form the backbone of ROS 2 communication architecture.

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. They are processes that perform computation and communicate with other nodes in the system.

**Key Characteristics of Nodes:**
- Encapsulate functionality in a single process
- Communicate with other nodes through topics, services, and actions
- Have unique names within a ROS graph
- Can be written in multiple programming languages (C++, Python, etc.)

**Creating a Node:**
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics - Publisher/Subscriber Pattern
Topics enable asynchronous communication between nodes using a publish/subscribe model. Publishers send messages to topics, and subscribers receive messages from topics.

**Topic Communication Features:**
- Unidirectional data flow from publisher to subscriber
- Multiple publishers and subscribers can use the same topic
- Data is sent continuously as long as publisher is active
- Latched topics retain last message for new subscribers

**Example Publisher:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example Subscriber:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services - Request/Response Pattern
Services provide synchronous communication between nodes using a request/response model. A client sends a request to a service, and the service sends back a response.

**Service Communication Features:**
- Synchronous communication (client waits for response)
- Request/response pattern
- One client to one service interaction
- Useful for actions that require a specific response

**Example Service Definition (add_two_ints.srv):**
```
int64 a
int64 b
---
int64 sum
```

**Example Service Server:**
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na={request.a}, b={request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example Service Client:**
```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) in ROS 2
QoS settings allow you to configure communication behavior for topics and services:

- **Reliability**: Reliable vs. best effort delivery
- **Durability**: Volatile vs. transient local (for latching)
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Maximum time between messages
- **Liveliness**: How to detect if publisher is alive

### Exercises and Questions

1. Explain the difference between nodes, topics, and services in ROS 2.
2. What is the publish/subscribe pattern and when should you use it?
3. Describe the request/response pattern and provide an example of when to use it.
4. Write a simple publisher node that publishes sensor data.
5. Create a subscriber node that processes incoming messages and logs them.
6. What is the purpose of Quality of Service (QoS) settings?
7. How do you handle multiple publishers and subscribers on the same topic?
8. Explain the lifecycle of a ROS 2 node.
9. What are the advantages of using topics over services?
10. When would you choose services over topics for communication?