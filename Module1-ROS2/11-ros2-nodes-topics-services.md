---
sidebar_position: 2
---

# ROS 2 Nodes, Topics, aur Services

## Chapter Title: ROS 2 Nodes, Topics, aur Services
**Roman Urdu Version:** ROS 2 nodes, topics, aur services

## Summary
Ye chapter ROS 2 ke core communication concepts ko detail mein cover karta hai jisme nodes, topics aur services shamil hain. Aap yahan se in components ke creation, implementation aur communication patterns ke bare mein practical knowledge hasil karenge.

Ye chapter ROS 2 ke communication architecture ka practical implementation hai. Nodes independent processes hote hain jo robot ke different functionalities ko handle karte hain. Topics pub/sub communication model provide karte hain aur services request/response communication provide karte hain.

## Learning Objectives
- ROS 2 nodes ke creation aur management ka tareeka seekhna
- Topics ke through pub/sub communication implement karna
- Services ke through request/response communication implement karna
- Communication patterns aur best practices ko samajhna

## Key Topics
1. Node Architecture aur Lifecycle
2. Publisher aur Subscriber Implementation
3. Message Types aur Interfaces
4. Service Servers aur Clients
5. Quality of Service (QoS) Settings
6. Parameter Management
7. Logging aur Debugging

## Chapter Content

### 1. Node Architecture aur Lifecycle

ROS 2 nodes ek independent process hote hain jo robot ke specific tasks ko perform karte hain. Har node apne lifecycle ke through ja sakta hai jo include karta hai:

- Unconfigured: Node loaded but not configured
- Inactive: Node configured but not active
- Active: Node running aur processing data
- Finalized: Node destroyed

Node lifecycle ke through resource management aur fault tolerance improve hoti hai.

### 2. Publisher aur Subscriber Implementation

Publishers messages ko specific topics par send karte hain aur subscribers in messages ko receive karte hain. Ye communication pattern asynchronous hoti hai jo loose coupling provide karti hai.

```cpp
// Example publisher implementation
auto publisher = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
```

```cpp
// Example subscriber implementation
auto subscription = this->create_subscription<std_msgs::msg::String>(
    "topic_name", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    });
```

### 3. Message Types aur Interfaces

ROS 2 messages strongly-typed data structures hote hain jo different nodes ke darmiyan communication ke liye use kiye jate hain. Messages .msg files mein define kiye jate hain aur different languages ke liye generate kiye jate hain.

Common message types:
- std_msgs: Basic data types
- geometry_msgs: Geometric data types
- sensor_msgs: Sensor data types
- nav_msgs: Navigation data types

### 4. Service Servers aur Clients

Services synchronous request/response communication provide karte hain jo specific operations ke liye suitable hote hain. Server request receive karta hai aur response send karta hai.

Service implementation mein:
- Service definition (.srv file)
- Server implementation
- Client implementation

### 5. Quality of Service (QoS) Settings

QoS profiles ROS 2 communication ke reliability, durability, history aur rate control ko define karte hain. Ye profiles different application requirements ke liye customize kiye jaa sakte hain.

Common QoS profiles:
- Reliability: Reliable ya Best effort
- Durability: Volatile ya Transient local
- History: Keep last ya Keep all
- Deadline aur Lifespan: Timing constraints

## Exercises and Questions

1. **Implementation:** Ek node banaiye jo ek topic par sensor data publish karta ho aur doosra node us data ko subscribe karta ho.

2. **Analysis:** Quality of Service (QoS) profiles ke different combinations ke effect ko explain kariye aur unke appropriate use cases bataiye.

3. **Design:** Ek robot navigation system ke liye nodes aur topics design kariye aur communication flow explain kariye.

4. **Problem Solving:** Agar ek subscriber messages nahi receive kar raha hai to troubleshooting steps kya honge?

5. **Comparison:** Publisher/subscriber aur service communication ke darmiyan when to use ka decision framework banaaiye.

6. **Coding:** Ek service server aur client pair implement kariye jo robot ke position ko set karne aur retrieve karne ke liye use kiya ja sake.

7. **Conceptual:** Node lifecycle states aur transitions ko explain kariye aur unke benefits bataiye.

8. **Scenario:** Real-time safety-critical system mein QoS settings kaise configure karenge aur kyun?

9. **Application:** Ek multi-robot system mein communication architecture design kariye aur nodes, topics aur services explain kariye.

10. **Research:** ROS 2 ke upcoming communication features aur security enhancements ke bare mein research kariye.