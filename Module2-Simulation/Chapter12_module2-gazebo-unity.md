---
sidebar_position: 2
title: "Module 2: Gazebo & Unity - Simulation, Physics, and Sensors"
---

# Module 2: Gazebo & Unity - Simulation, Physics, and Sensors

## Learning Objectives

By the end of this module, you will be able to:
- Set up and configure Gazebo simulation environments for robotics
- Integrate Unity with ROS 2 for advanced simulation scenarios
- Model and simulate various sensor types (LIDAR, cameras, IMU, etc.)
- Understand physics simulation principles and their impact on robot behavior
- Create custom simulation worlds with obstacles and interactive elements
- Debug and optimize simulation performance

## Introduction to Simulation in Robotics

Simulation plays a crucial role in robotics development, allowing engineers and researchers to test algorithms, validate designs, and train AI systems without the risks and costs associated with physical hardware. In the context of Physical AI and humanoid robotics, simulation provides a safe environment to experiment with complex behaviors and interactions.

### Why Simulation is Critical

1. **Safety**: Test dangerous or unpredictable behaviors without risk to hardware or humans
2. **Cost-Effectiveness**: Develop and test without expensive physical hardware
3. **Repeatability**: Create consistent testing conditions for algorithm validation
4. **Speed**: Run experiments faster than real-time for rapid iteration
5. **Scalability**: Test with multiple robots simultaneously
6. **Environment Variety**: Test in diverse environments without physical constraints

### Simulation vs. Reality Gap

One of the biggest challenges in robotics simulation is the "reality gap" - the difference between simulated and real-world behavior. Modern simulation tools like Gazebo and Unity have made significant strides in minimizing this gap through:
- Accurate physics engines
- Realistic sensor models
- Detailed material properties
- Environmental effects simulation

## Gazebo: Physics-Based Simulation

### Overview of Gazebo

Gazebo is a physics-based simulation environment that provides realistic dynamics, sensor simulation, and rendering capabilities. It's widely used in the robotics community and integrates seamlessly with ROS 2.

### Installing Gazebo

For Ubuntu 22.04 with ROS 2 Humble:
```bash
# Install Gazebo Garden (recommended version for ROS 2 Humble)
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### Basic Gazebo Concepts

1. **Worlds**: Environments that contain models, lighting, and physics properties
2. **Models**: 3D objects that represent robots, obstacles, and other entities
3. **Sensors**: Simulated devices that provide data to your robot
4. **Plugins**: Extensions that add functionality to models or the simulation
5. **Physics Engine**: Underlying system that calculates forces, collisions, and dynamics

### Creating Your First Gazebo World

Let's create a simple world file. Create the directory structure:

```bash
mkdir -p ~/ros2_ws/src/my_robot_bringup/worlds
```

Create the world file `~/ros2_ws/src/my_robot_bringup/worlds/simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Define physics properties -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Add a simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Add a simple cylinder obstacle -->
    <model name="cylinder_obstacle">
      <pose>-2 1 0.5 0 0 0</pose>
      <link name="cylinder_link">
        <collision name="cylinder_collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.145833</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.145833</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Spawning Your Robot in Gazebo

Let's create a launch file to spawn our robot in the custom world. Update the launch file `~/ros2_ws/src/my_robot_bringup/launch/robot_spawn_with_world.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_robot_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(pkg_share, 'worlds', 'simple_world.world'))
    
    # Get URDF file path
    urdf_file_path = os.path.join(pkg_share, 'urdf/simple_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()
    
    # Create the Gazebo server node
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Create the Gazebo client node
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen'
    )
    
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
    
    # Create the joint state publisher node
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
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
```

### Adding Sensors to Your Robot

Let's enhance our robot model with various sensors. Create an updated URDF file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/urdf/robot_with_sensors.urdf`:

```xml
<?xml version="1.0"?>
<robot name="robot_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
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

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Laser scanner link -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- IMU link -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
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

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0.0 0.1" rpy="0 0 0"/>
  </joint>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.2 0.0 0.05" rpy="0 0 0"/>
  </joint>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for differential drive and sensors -->
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

  <!-- Camera sensor plugin -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Laser scanner plugin -->
  <gazebo reference="laser_link">
    <sensor name="laser" type="ray">
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
        <frame_name>laser_link</frame_name>
        <topic_name>scan</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor plugin -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <frame_name>imu_link</frame_name>
        <topic_name>imu</topic_name>
      </plugin>
    </sensor>
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
  
  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>
  
  <gazebo reference="laser_link">
    <material>Gazebo/Green</material>
  </gazebo>
</robot>
```

### Sensor Data Processing Node

Create a node to process sensor data from the simulated robot. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/sensor_processor.py`:

```python
#!/usr/bin/env python3
"""
Sensor processor node that subscribes to multiple sensor topics.
This node demonstrates sensor data processing in simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class SensorProcessor(Node):
    """
    A node that processes data from multiple simulated sensors.
    """
    
    def __init__(self):
        # Initialize the node with the name 'sensor_processor'
        super().__init__('sensor_processor')
        
        # Create subscriptions for different sensor topics
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize sensor data storage
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None
        self.latest_odom = None
        
        # Initialize obstacle avoidance state
        self.obstacle_detected = False
        
        # Create a timer for obstacle avoidance behavior
        self.timer = self.create_timer(0.1, self.obstacle_avoidance_behavior)
        
        # Log that the sensor processor has started
        self.get_logger().info('Sensor Processor has started')
    
    def scan_callback(self, msg):
        """
        Callback method for laser scan messages.
        """
        self.latest_scan = msg
        
        # Process the scan data
        if len(msg.ranges) > 0:
            # Find the closest distance in the front 90 degrees
            front_ranges = msg.ranges[len(msg.ranges)//2 - 45 : len(msg.ranges)//2 + 45]
            front_min = min(r for r in front_ranges if not np.isnan(r) and r > 0)
            
            # Log obstacle detection
            if front_min < 1.0:
                self.obstacle_detected = True
                self.get_logger().warn(f'Obstacle detected! Distance: {front_min:.2f}m')
            else:
                self.obstacle_detected = False
    
    def image_callback(self, msg):
        """
        Callback method for image messages.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store the image for processing
            self.latest_image = cv_image
            
            # Perform basic image processing (edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            
            # Display the processed image
            cv2.imshow("Camera Feed", cv_image)
            cv2.imshow("Edges", edges)
            cv2.waitKey(1)  # Process GUI events
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def imu_callback(self, msg):
        """
        Callback method for IMU messages.
        """
        self.latest_imu = msg
        
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        
        # Log IMU data
        self.get_logger().info(
            f'IMU - Roll: {self.quaternion_to_roll_pitch_yaw(orientation)[0]:.2f}, '
            f'Pitch: {self.quaternion_to_roll_pitch_yaw(orientation)[1]:.2f}, '
            f'Yaw: {self.quaternion_to_roll_pitch_yaw(orientation)[2]:.2f}'
        )
    
    def odom_callback(self, msg):
        """
        Callback method for odometry messages.
        """
        self.latest_odom = msg
        
        # Extract position and velocity
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        
        # Log odometry data
        self.get_logger().info(
            f'Odom - X: {position.x:.2f}, Y: {position.y:.2f}, '
            f'Linear Vel: {velocity.x:.2f}'
        )
    
    def obstacle_avoidance_behavior(self):
        """
        Simple obstacle avoidance behavior based on laser scan data.
        """
        if self.latest_scan is None:
            return
        
        # Get the front scan ranges
        front_ranges = self.latest_scan.ranges[len(self.latest_scan.ranges)//2 - 45 : len(self.latest_scan.ranges)//2 + 45]
        front_min = min(r for r in front_ranges if not np.isnan(r) and r > 0)
        
        # Create twist message for velocity commands
        twist_msg = Twist()
        
        if front_min < 1.0:
            # Obstacle detected - turn right
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.5  # Turn right
            self.get_logger().info('Avoiding obstacle - turning right')
        else:
            # Clear path - move forward
            twist_msg.linear.x = 0.5  # Move forward
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        
        # Publish the velocity command
        self.cmd_vel_pub.publish(twist_msg)
    
    def quaternion_to_roll_pitch_yaw(self, quaternion):
        """
        Convert quaternion to roll, pitch, yaw angles.
        """
        import math
        
        # Extract quaternion components
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Convert to roll, pitch, yaw
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the SensorProcessor node
    sensor_processor = SensorProcessor()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        sensor_processor.get_logger().info('Shutting down sensor processor...')
    finally:
        # Destroy OpenCV windows
        cv2.destroyAllWindows()
        
        # Destroy the node explicitly
        sensor_processor.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update the setup.py to include the new node and dependencies:

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
        ('share/' + package_name + '/urdf', [
            'my_robot_bringup/urdf/simple_robot.urdf',
            'my_robot_bringup/urdf/robot_with_sensors.urdf'
        ]),
        ('share/' + package_name + '/worlds', ['my_robot_bringup/worlds/simple_world.world']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
        ('share/' + package_name + '/msg', ['my_robot_bringup/msg/BatteryStatus.msg']),
        ('share/' + package_name + '/launch', [
            'launch/robot_spawn.launch.py',
            'launch/robot_spawn_with_world.launch.py'
        ]),
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
            'sensor_processor = my_robot_bringup.sensor_processor:main',
        ],
    },
)
```

### Running the Simulation with Sensors

Now let's build and run the simulation with sensors:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash

# Launch the robot with sensors in the custom world
ros2 launch my_robot_bringup robot_spawn_with_world.launch.py
```

In another terminal, run the sensor processor:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_bringup sensor_processor
```

## Unity Integration for Robotics

### Overview of Unity for Robotics

Unity is a powerful game engine that has been adapted for robotics simulation. The Unity Robotics Hub provides tools and packages specifically designed for robotics applications, including:
- Physics simulation with realistic collision detection
- High-quality graphics rendering
- Integration with ROS 2 through the ROS# package
- Support for various sensor types
- VR/AR capabilities for immersive simulation

### Setting up Unity for Robotics

While Unity is primarily a Windows/Mac application, we can demonstrate the concepts and integration patterns that would be used in a Unity-ROS2 setup.

### Unity-ROS2 Communication

The Unity Robotics Hub uses the ROS# communication library to connect Unity with ROS 2. Here's how the communication works:

1. **ROS# Client**: Runs in Unity and communicates with ROS 2
2. **Message Bridge**: Translates between Unity and ROS 2 message formats
3. **ROS 2 Bridge**: Standard ROS 2 nodes that interface with Unity

### Example Unity-ROS2 Communication Node

Create a node that demonstrates the communication patterns that would be used with Unity. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/unity_bridge.py`:

```python
#!/usr/bin/env python3
"""
Unity bridge node that simulates Unity-ROS2 communication patterns.
This node demonstrates how Unity would interface with ROS 2.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import random


class UnityBridge(Node):
    """
    A node that simulates Unity-ROS2 communication patterns.
    """
    
    def __init__(self):
        # Initialize the node with the name 'unity_bridge'
        super().__init__('unity_bridge')
        
        # Create publishers for Unity visualization
        self.marker_pub = self.create_publisher(Marker, 'unity_visualization', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'unity_marker_array', 10)
        
        # Create publishers for Unity robot state
        self.joint_state_pub = self.create_publisher(JointState, 'unity_joint_states', 10)
        self.robot_pose_pub = self.create_publisher(Pose, 'unity_robot_pose', 10)
        
        # Create subscribers for Unity commands
        self.unity_cmd_vel_sub = self.create_subscription(
            Twist,
            'unity_cmd_vel',
            self.unity_cmd_vel_callback,
            10)
        
        # Create a timer for publishing Unity data
        self.timer = self.create_timer(0.1, self.publish_unity_data)
        
        # Initialize robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_linear_vel = 0.0
        self.robot_angular_vel = 0.0
        
        # Initialize visualization counter
        self.marker_id = 0
        
        # Log that the Unity bridge has started
        self.get_logger().info('Unity Bridge has started')
    
    def unity_cmd_vel_callback(self, msg):
        """
        Callback for Unity velocity commands.
        """
        self.robot_linear_vel = msg.linear.x
        self.robot_angular_vel = msg.angular.z
        
        # Update robot position based on velocity
        dt = 0.1  # Assuming 10Hz update rate
        self.robot_x += self.robot_linear_vel * math.cos(self.robot_theta) * dt
        self.robot_y += self.robot_linear_vel * math.sin(self.robot_theta) * dt
        self.robot_theta += self.robot_angular_vel * dt
        
        self.get_logger().info(
            f'Unity command received - Linear: {msg.linear.x:.2f}, '
            f'Angular: {msg.angular.z:.2f}'
        )
    
    def publish_unity_data(self):
        """
        Publish Unity-compatible data.
        """
        # Publish joint states (simulating Unity robot joints)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [
            self.robot_theta * 2,  # Simulate wheel rotation based on robot rotation
            self.robot_theta * 2
        ]
        joint_state_msg.velocity = [self.robot_linear_vel * 2, self.robot_linear_vel * 2]
        joint_state_msg.effort = [0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state_msg)
        
        # Publish robot pose
        pose_msg = Pose()
        pose_msg.position.x = self.robot_x
        pose_msg.position.y = self.robot_y
        pose_msg.position.z = 0.1  # Height above ground
        # Convert theta to quaternion
        pose_msg.orientation.z = math.sin(self.robot_theta / 2.0)
        pose_msg.orientation.w = math.cos(self.robot_theta / 2.0)
        
        self.robot_pose_pub.publish(pose_msg)
        
        # Publish visualization markers
        self.publish_visualization_markers()
    
    def publish_visualization_markers(self):
        """
        Publish visualization markers for Unity.
        """
        # Create a marker array
        marker_array = MarkerArray()
        
        # Create a robot marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "unity_robots"
        robot_marker.id = 0
        robot_marker.type = Marker.CUBE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.075  # Half of robot height
        robot_marker.pose.orientation.x = 0.0
        robot_marker.pose.orientation.y = 0.0
        robot_marker.pose.orientation.z = math.sin(self.robot_theta / 2.0)
        robot_marker.pose.orientation.w = math.cos(self.robot_theta / 2.0)
        robot_marker.scale.x = 0.5
        robot_marker.scale.y = 0.3
        robot_marker.scale.z = 0.15
        robot_marker.color.a = 1.0
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        
        marker_array.markers.append(robot_marker)
        
        # Create a trajectory marker
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.ns = "unity_trajectory"
        trajectory_marker.id = 1
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        
        # Add points to the trajectory (last 10 positions)
        for i in range(10):
            point = trajectory_marker.points.add()
            point.x = self.robot_x - i * 0.1 * math.cos(self.robot_theta)
            point.y = self.robot_y - i * 0.1 * math.sin(self.robot_theta)
            point.z = 0.075
        
        trajectory_marker.scale.x = 0.05
        trajectory_marker.color.a = 0.8
        trajectory_marker.color.r = 1.0
        trajectory_marker.color.g = 0.0
        trajectory_marker.color.b = 0.0
        
        marker_array.markers.append(trajectory_marker)
        
        # Publish the marker array
        self.marker_array_pub.publish(marker_array)
        
        # Create an interactive object marker
        obj_marker = Marker()
        obj_marker.header.frame_id = "map"
        obj_marker.header.stamp = self.get_clock().now().to_msg()
        obj_marker.ns = "unity_objects"
        obj_marker.id = 2
        obj_marker.type = Marker.CYLINDER
        obj_marker.action = Marker.ADD
        obj_marker.pose.position.x = 2.0
        obj_marker.pose.position.y = 1.0
        obj_marker.pose.position.z = 0.5
        obj_marker.pose.orientation.x = 0.0
        obj_marker.pose.orientation.y = 0.0
        obj_marker.pose.orientation.z = 0.0
        obj_marker.pose.orientation.w = 1.0
        obj_marker.scale.x = 1.0
        obj_marker.scale.y = 1.0
        obj_marker.scale.z = 1.0
        obj_marker.color.a = 1.0
        obj_marker.color.r = 0.0
        obj_marker.color.g = 1.0
        obj_marker.color.b = 0.0
        
        self.marker_pub.publish(obj_marker)


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the UnityBridge node
    unity_bridge = UnityBridge()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        unity_bridge.get_logger().info('Shutting down Unity bridge...')
    finally:
        # Destroy the node explicitly
        unity_bridge.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update the setup.py to include the Unity bridge:

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
        ('share/' + package_name + '/urdf', [
            'my_robot_bringup/urdf/simple_robot.urdf',
            'my_robot_bringup/urdf/robot_with_sensors.urdf'
        ]),
        ('share/' + package_name + '/worlds', ['my_robot_bringup/worlds/simple_world.world']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
        ('share/' + package_name + '/msg', ['my_robot_bringup/msg/BatteryStatus.msg']),
        ('share/' + package_name + '/launch', [
            'launch/robot_spawn.launch.py',
            'launch/robot_spawn_with_world.launch.py'
        ]),
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
            'sensor_processor = my_robot_bringup.sensor_processor:main',
            'unity_bridge = my_robot_bringup.unity_bridge:main',
        ],
    },
)
```

## Advanced Physics Simulation

### Understanding Physics Engines

Physics engines in simulation environments calculate the motion, collisions, and forces acting on objects. Key physics concepts in robotics simulation include:

1. **Collision Detection**: Determining when objects intersect
2. **Rigid Body Dynamics**: Calculating motion based on forces and torques
3. **Constraints**: Limiting motion (like joints)
4. **Friction and Contact**: Modeling surface interactions

### Physics Parameters Tuning

Here's a node that demonstrates physics parameter tuning. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/physics_tuner.py`:

```python
#!/usr/bin/env python3
"""
Physics tuner node that demonstrates physics parameter adjustment.
This node shows how to tune physics parameters for realistic simulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class PhysicsTuner(Node):
    """
    A node that demonstrates physics parameter tuning.
    """
    
    def __init__(self):
        # Initialize the node with the name 'physics_tuner'
        super().__init__('physics_tuner')
        
        # Create publishers for physics parameters
        self.friction_pub = self.create_publisher(Float32, 'surface_friction', 10)
        self.damping_pub = self.create_publisher(Float32, 'damping_factor', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriptions for sensor feedback
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Initialize physics parameters
        self.surface_friction = 0.5
        self.damping_factor = 0.1
        self.simulation_mode = 0  # 0: normal, 1: low friction, 2: high damping
        
        # Create a timer for physics parameter updates
        self.timer = self.create_timer(2.0, self.update_physics_parameters)
        
        # Create a timer for robot behavior
        self.behavior_timer = self.create_timer(0.1, self.robot_behavior)
        
        # Initialize robot state
        self.robot_speed = 0.0
        self.obstacle_detected = False
        
        # Log that the physics tuner has started
        self.get_logger().info('Physics Tuner has started')
    
    def scan_callback(self, msg):
        """
        Callback for laser scan to detect obstacles.
        """
        if len(msg.ranges) > 0:
            # Check front ranges
            front_ranges = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
            front_min = min(r for r in front_ranges if not math.isnan(r) and r > 0)
            
            self.obstacle_detected = front_min < 1.0
    
    def update_physics_parameters(self):
        """
        Update physics parameters based on simulation mode.
        """
        # Cycle through different physics modes
        self.simulation_mode = (self.simulation_mode + 1) % 3
        
        if self.simulation_mode == 0:
            # Normal mode
            self.surface_friction = 0.5
            self.damping_factor = 0.1
            mode_name = "Normal"
        elif self.simulation_mode == 1:
            # Low friction mode (icy surface)
            self.surface_friction = 0.1
            self.damping_factor = 0.05
            mode_name = "Low Friction (Icy)"
        else:
            # High damping mode (viscous liquid)
            self.surface_friction = 0.8
            self.damping_factor = 0.5
            mode_name = "High Damping (Viscous)"
        
        # Publish the new parameters
        friction_msg = Float32()
        friction_msg.data = self.surface_friction
        self.friction_pub.publish(friction_msg)
        
        damping_msg = Float32()
        damping_msg.data = self.damping_factor
        self.damping_pub.publish(damping_msg)
        
        self.get_logger().info(
            f'Physics parameters updated - Mode: {mode_name}, '
            f'Friction: {self.surface_friction:.2f}, '
            f'Damping: {self.damping_factor:.2f}'
        )
    
    def robot_behavior(self):
        """
        Simple robot behavior that responds to physics changes.
        """
        twist_msg = Twist()
        
        if self.obstacle_detected:
            # Stop when obstacle detected
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            # Adjust speed based on surface friction
            base_speed = 0.5
            adjusted_speed = base_speed * (1.0 - self.damping_factor * 0.5)
            
            # On low friction surfaces, reduce speed to prevent sliding
            if self.surface_friction < 0.3:
                adjusted_speed *= 0.7
            
            twist_msg.linear.x = adjusted_speed
            twist_msg.angular.z = 0.0  # Move straight
        
        self.cmd_vel_pub.publish(twist_msg)
        
        self.get_logger().info(
            f'Robot behavior - Speed: {twist_msg.linear.x:.2f}, '
            f'Surface friction: {self.surface_friction:.2f}'
        )


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the PhysicsTuner node
    physics_tuner = PhysicsTuner()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(physics_tuner)
    except KeyboardInterrupt:
        physics_tuner.get_logger().info('Shutting down physics tuner...')
    finally:
        # Destroy the node explicitly
        physics_tuner.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Sensor Simulation in Detail

### Types of Sensors in Simulation

Different sensors require different simulation approaches:

1. **LIDAR**: Simulated using ray tracing in the physics engine
2. **Cameras**: Rendered using the graphics engine
3. **IMU**: Simulated with noise models added to perfect data
4. **Encoders**: Derived from simulated joint positions
5. **Force/Torque**: Calculated from simulated contact forces

### Advanced Sensor Fusion Node

Create a comprehensive sensor fusion node that combines data from multiple sensors. Create the file `~/ros2_ws/src/my_robot_bringup/my_robot_bringup/sensor_fusion.py`:

```python
#!/usr/bin/env python3
"""
Advanced sensor fusion node that combines data from multiple sensors.
This node demonstrates Kalman filtering and sensor fusion techniques.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class SensorFusion(Node):
    """
    A node that performs sensor fusion using Kalman filtering.
    """
    
    def __init__(self):
        # Initialize the node with the name 'sensor_fusion'
        super().__init__('sensor_fusion')
        
        # Create subscriptions for all sensor types
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Create publishers for fused data
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'fused_pose', 10)
        self.fused_twist_pub = self.create_publisher(Twist, 'fused_twist', 10)
        
        # Initialize sensor data storage
        self.latest_scan = None
        self.latest_imu = None
        self.latest_joint = None
        self.latest_odom = None
        
        # Initialize Kalman filter state
        # State: [x, y, theta, vx, vy, omega]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.eye(6) * 1000.0  # High initial uncertainty
        
        # Process noise covariance
        self.Q = np.diag([0.1, 0.1, 0.01, 0.5, 0.5, 0.1])  # Process noise
        
        # Measurement noise covariances
        self.R_odom = np.diag([0.01, 0.01, 0.001])  # Position measurement noise
        self.R_imu = np.diag([0.01, 0.01, 0.001])   # Orientation measurement noise
        self.R_enc = np.diag([0.1, 0.1, 0.05])      # Velocity measurement noise
        
        # Initialize time tracking
        self.last_time = self.get_clock().now()
        
        # Create a timer for prediction step
        self.timer = self.create_timer(0.05, self.prediction_step)  # 20 Hz
        
        # Log that the sensor fusion has started
        self.get_logger().info('Sensor Fusion has started')
    
    def scan_callback(self, msg):
        """
        Callback for laser scan data.
        """
        self.latest_scan = msg
        
        # Extract useful information from scan
        if len(msg.ranges) > 0:
            # Find obstacles and calculate distances
            front_ranges = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
            front_min = min(r for r in front_ranges if not math.isnan(r) and r > 0)
            
            # Use scan data to update position uncertainty
            # If we see consistent landmarks, reduce position uncertainty
            if front_min > 2.0:  # Clear path
                # Reduce position uncertainty in the direction of movement
                self.covariance[0, 0] *= 0.99  # Reduce x uncertainty
                self.covariance[1, 1] *= 0.99  # Reduce y uncertainty
    
    def imu_callback(self, msg):
        """
        Callback for IMU data.
        """
        self.latest_imu = msg
        
        # Extract orientation from quaternion
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        euler = r.as_euler('xyz')
        
        # Measurement vector [theta, omega_z]
        z = np.array([euler[2], msg.angular_velocity.z])
        
        # Measurement matrix for orientation and angular velocity
        H = np.array([
            [0, 0, 1, 0, 0, 0],  # theta measurement
            [0, 0, 0, 0, 0, 1]   # omega measurement
        ])
        
        # Measurement covariance
        R = self.R_imu
        
        # Update step
        self.update_step(z, H, R)
    
    def joint_callback(self, msg):
        """
        Callback for joint state data (encoders).
        """
        self.latest_joint = msg
        
        # Calculate velocity from joint positions
        if 'left_wheel_joint' in msg.name and 'right_wheel_joint' in msg.name:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            # Calculate linear and angular velocity from wheel encoders
            wheel_radius = 0.05
            wheel_separation = 0.35
            
            left_vel = msg.velocity[left_idx] if len(msg.velocity) > left_idx else 0.0
            right_vel = msg.velocity[right_idx] if len(msg.velocity) > right_idx else 0.0
            
            # Convert wheel velocities to robot velocities
            linear_vel = (left_vel + right_vel) * wheel_radius / 2.0
            angular_vel = (right_vel - left_vel) * wheel_radius / wheel_separation
            
            # Measurement vector [vx, vy, omega]
            z = np.array([linear_vel * math.cos(self.state[2]), 
                         linear_vel * math.sin(self.state[2]), 
                         angular_vel])
            
            # Measurement matrix for velocity
            H = np.array([
                [0, 0, 0, 1, 0, 0],  # vx measurement
                [0, 0, 0, 0, 1, 0],  # vy measurement
                [0, 0, 0, 0, 0, 1]   # omega measurement
            ])
            
            # Measurement covariance
            R = self.R_enc
            
            # Update step
            self.update_step(z, H, R)
    
    def odom_callback(self, msg):
        """
        Callback for odometry data.
        """
        self.latest_odom = msg
        
        # Extract position and orientation from odometry
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        
        # Convert quaternion to euler
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = r.as_euler('xyz')
        
        # Measurement vector [x, y, theta]
        z = np.array([pos.x, pos.y, euler[2]])
        
        # Measurement matrix for position and orientation
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # x measurement
            [0, 1, 0, 0, 0, 0],  # y measurement
            [0, 0, 1, 0, 0, 0]   # theta measurement
        ])
        
        # Measurement covariance
        R = self.R_odom
        
        # Update step
        self.update_step(z, H, R)
    
    def prediction_step(self):
        """
        Prediction step of the Kalman filter.
        """
        # Get current time for dt calculation
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # State transition model (constant velocity model)
        F = np.array([
            [1, 0, 0, dt, 0, 0],      # x = x + vx*dt
            [0, 1, 0, 0, dt, 0],      # y = y + vy*dt
            [0, 0, 1, 0, 0, dt],      # theta = theta + omega*dt
            [0, 0, 0, 1, 0, 0],       # vx = vx (constant)
            [0, 0, 0, 0, 1, 0],       # vy = vy (constant)
            [0, 0, 0, 0, 0, 1]        # omega = omega (constant)
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q * dt
        
        # Normalize angle to [-pi, pi]
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Publish the predicted state
        self.publish_fused_state()
    
    def update_step(self, z, H, R):
        """
        Update step of the Kalman filter.
        """
        # Calculate innovation
        innovation = z - H @ self.state[:len(z)]
        
        # Normalize angle innovation if it's an orientation measurement
        if len(innovation) > 2:  # If orientation is included
            innovation[2] = math.atan2(math.sin(innovation[2]), math.cos(innovation[2]))
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + R
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state[:len(z)] += K @ innovation
        
        # Update covariance
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance
        
        # Normalize angle after update
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
    
    def publish_fused_state(self):
        """
        Publish the fused state estimate.
        """
        # Create and publish pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert angle to quaternion
        q = R.from_euler('xyz', [0, 0, self.state[2]]).as_quat()
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]
        
        # Flatten covariance matrix
        pose_msg.pose.covariance = self.covariance[:6, :6].flatten().tolist()
        
        self.fused_pose_pub.publish(pose_msg)
        
        # Create and publish twist
        twist_msg = Twist()
        twist_msg.linear.x = self.state[3]  # vx
        twist_msg.linear.y = self.state[4]  # vy
        twist_msg.angular.z = self.state[5]  # omega
        
        self.fused_twist_pub.publish(twist_msg)
        
        self.get_logger().info(
            f'Fused State - X: {self.state[0]:.2f}, Y: {self.state[1]:.2f}, '
            f'Theta: {math.degrees(self.state[2]):.2f}°, '
            f'V: ({self.state[3]:.2f}, {self.state[4]:.2f}), '
            f'Omega: {self.state[5]:.2f}'
        )


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the SensorFusion node
    sensor_fusion = SensorFusion()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        sensor_fusion.get_logger().info('Shutting down sensor fusion...')
    finally:
        # Destroy the node explicitly
        sensor_fusion.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update the setup.py to include the new nodes:

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
        ('share/' + package_name + '/urdf', [
            'my_robot_bringup/urdf/simple_robot.urdf',
            'my_robot_bringup/urdf/robot_with_sensors.urdf'
        ]),
        ('share/' + package_name + '/worlds', ['my_robot_bringup/worlds/simple_world.world']),
        ('share/' + package_name + '/srv', ['my_robot_bringup/srv/MoveRobot.srv']),
        ('share/' + package_name + '/action', ['my_robot_bringup/action/Navigation.action']),
        ('share/' + package_name + '/msg', ['my_robot_bringup/msg/BatteryStatus.msg']),
        ('share/' + package_name + '/launch', [
            'launch/robot_spawn.launch.py',
            'launch/robot_spawn_with_world.launch.py'
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'opencv-python'],
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
            'sensor_processor = my_robot_bringup.sensor_processor:main',
            'unity_bridge = my_robot_bringup.unity_bridge:main',
            'physics_tuner = my_robot_bringup.physics_tuner:main',
            'sensor_fusion = my_robot_bringup.sensor_fusion:main',
        ],
    },
)
```

## Exercises and Mini Projects

### Exercise 1: Custom World with Multiple Robots

Create a Gazebo world with multiple robots that interact with each other.

**Solution:**

Create the multi-robot world file `~/ros2_ws/src/my_robot_bringup/worlds/multi_robot_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_robot_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Define physics properties -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Robot 1 -->
    <model name="robot1">
      <include>
        <uri>model://simple_robot</uri>
        <pose>0 0 0.1 0 0 0</pose>
      </include>
    </model>
    
    <!-- Robot 2 -->
    <model name="robot2">
      <include>
        <uri>model://simple_robot</uri>
        <pose>2 2 0.1 0 0 1.57</pose>
      </include>
    </model>
    
    <!-- Robot 3 -->
    <model name="robot3">
      <include>
        <uri>model://simple_robot</uri>
        <pose>-2 -2 0.1 0 0 -1.57</pose>
      </include>
    </model>
    
    <!-- Obstacles -->
    <model name="wall1">
      <pose>0 4 0.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>8 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>8 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Mini Project: Swarm Robotics Simulation

Create a system where multiple robots coordinate to achieve a common goal, such as mapping an unknown environment or transporting an object together.

This project would involve creating multiple robot instances that communicate with each other to coordinate their actions, demonstrating advanced simulation concepts.

## Summary

In this module, we've covered:

1. **Gazebo Simulation**: Creating realistic physics-based environments
2. **Unity Integration**: Understanding Unity-ROS2 communication patterns
3. **Sensor Simulation**: Modeling various sensor types with realistic noise
4. **Physics Tuning**: Adjusting parameters for realistic behavior
5. **Sensor Fusion**: Combining multiple sensor inputs for better state estimation

These simulation skills are crucial for developing and testing robotics algorithms before deploying them on physical hardware. The ability to create realistic simulations that accurately represent the real world is essential for successful robotics development.

In the next module, we'll explore NVIDIA Isaac, which provides advanced tools for robotics development with GPU acceleration and AI integration.