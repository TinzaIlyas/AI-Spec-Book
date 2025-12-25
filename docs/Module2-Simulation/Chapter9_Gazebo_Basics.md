# Chapter 2: Gazebo Basics

## Gazebo Install Aur Setup

Gazebo ko install karne ke liye humein system requirements ko fulfill karna hoga aur sahi version install karni hogi. Gazebo ROS ke sath well-integrated hai, isliye ROS installation ke sath Gazebo automatically install ho jata hai.

Ubuntu/Linux par installation command:
```bash
sudo apt-get install gazebo libgazebo-dev
```

Windows aur macOS par installation slightly different hoti hai aur official documentation follow karni chahiye.

Post-installation steps:
- Environment variables set karna
- Plugin paths configure karna
- Basic launch test karna

## URDF Aur SDF Robot Description

### URDF (Unified Robot Description Format)

URDF XML-based format hai jo robot ka structure, joints, links aur sensors describe karta hai. URDF me har robot part ko link kehte hain aur joints in links ko connect karte hain.

Basic URDF structure me ye elements hote hain:
- Links: Robot ka rigid body parts
- Joints: Links ko connect karne wala mechanism
- Visual: Robot ka visual representation
- Collision: Collision detection ke liye geometry
- Inertial: Mass, center of mass, inertia properties

### SDF (Simulation Description Format)

SDF Gazebo ke liye native format hai aur URDF se zyada features support karta hai. SDF me complete simulation environment describe kiya jata hai including robots, sensors, objects aur environment.

URDF ko SDF me convert kiya ja sakta hai Gazebo ke andhar use karne ke liye.

## Physics Simulation Basics (Gravity, Friction, Collisions)

### Gravity

Gravity simulation real-world physics ko mimic karne ke liye zaruri hai. Gazebo me gravity value change kiya ja sakta hai different scenarios ke liye. Default value -9.81 m/s² hota hai.

### Friction

Friction robot surfaces aur environment ke beech interaction ko define karta hai. Two types hain:
- Static friction: Object ko move hone se rokta hai
- Dynamic friction: Moving object par effect hota hai

### Collisions

Collision detection robot safety aur realistic interaction ke liye zaruri hai. Gazebo different collision algorithms provide karta hai:
- Bullet collision engine
- ODE collision engine
- FCL collision engine

## Simple Robot Model Create Karna

Simple robot model banane ke liye hum basic URDF file create karte hain. Hum ek differential drive robot ka example le sakte hain jo 2 wheels aur 1 caster wheel se bana hota hai.

Basic steps:
1. Base link define karna
2. Wheel links add karna
3. Joints create karna
4. Visual aur collision properties define karna
5. URDF ko Gazebo compatible banane ke liye gazebo tags add karna

# Subtopics

## URDF vs SDF

| URDF | SDF |
|------|-----|
| ROS ke liye primary format | Gazebo ke liye native format |
| Robot description par focus | Complete simulation environment |
| Limited simulation features | Advanced simulation features |
| XML-based | XML-based |

## Gazebo World Setup

Gazebo world setup me environment creation shamil hota hai jisme:

- Static objects placement
- Lighting configuration
- Camera setup
- Physics engine selection
- World boundaries

World file (.world) XML format me hota hai aur Gazebo environment ko define karta hai.

## Sensors Simulate Karna (LiDAR, Camera, IMU)

### LiDAR Sensor

LiDAR line ya point clouds generate karta hai obstacles detect karne ke liye. Configuration parameters me:
- Range
- Resolution
- Field of view
- Scan rate

### Camera Sensor

Camera realistic image data provide karta hai. Parameters include:
- Resolution
- Field of view
- Image format
- Noise model

### IMU Sensor

IMU (Inertial Measurement Unit) acceleration, angular velocity aur orientation data provide karta hai.

# Exercises / Questions

1. Apne simple robot ka URDF file banao aur Gazebo me load karo
2. Robot ke sensors ko simulate karo aur data observe karo
3. Physics parameters (mass, friction) change karke robot behavior observe karo