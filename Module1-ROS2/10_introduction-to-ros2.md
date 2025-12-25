---
sidebar_position: 1
---

# ROS 2 Ka Taaruf: Robot Operating System 2

## Chapter Title: ROS 2 Ka Taaruf: Robot Operating System 2
**Roman Urdu Version:** ROS 2 ka taaruf: Robot Operating System 2

## Summary
Ye chapter ROS 2 ke basic concepts ko cover karta hai, jis mein ROS 1 se ROS 2 tak ka transition, architecture aur core components shamil hain. Aap yahan se ROS 2 ke fundamental concepts, nodes, topics, services aur actions ke bare mein seekhenge.

ROS 2 (Robot Operating System 2) ek flexible framework hai jo robotics software development ke liye use kiya jata hai. Ye ek middleware hai jo communication, hardware abstraction, device drivers, libraries aur visualization tools provide karta hai jo robotics applications banane ke liye zaroori hain.

## Learning Objectives
- ROS 2 ke architecture aur design principles ko samajhna
- ROS 1 aur ROS 2 ke darmiyan farq samajhna
- ROS 2 ecosystem ke components ko samajhna
- ROS 2 ke installation aur configuration ka tareeka seekhna

## Key Topics
1. ROS 2 Introduction aur Background
2. ROS 1 vs ROS 2 Comparison
3. DDS (Data Distribution Service) aur Communication Architecture
4. Nodes, Topics, Services aur Actions
5. ROS 2 Packages aur Workspaces
6. Launch Files aur Parameters

## Chapter Content

### 1. ROS 2 Introduction aur Background

ROS 2 ko open-source robotics community ke requirements aur feedback ke hisab se develop kiya gaya tha. ROS 1 ki security, real-time performance aur deployment ke kuchh limitations thi jo ROS 2 mein resolve ki gayi hain.

ROS 2 mein sabse bara improvement real-time applications aur production environments ke liye suitable hone ka hai. Ye DDS (Data Distribution Service) protocol ke upar build kiya gaya hai jo reliable, real-time data exchange provide karta hai.

### 2. ROS 1 vs ROS 2 Comparison

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS-based |
| Security | Limited | Built-in Security |
| Real-time Support | Limited | Enhanced |
| Cross-platform | Linux-focused | Multi-platform |
| Quality of Service | Limited | Advanced QoS |

### 3. DDS (Data Distribution Service) aur Communication Architecture

DDS ek middleware hai jo real-time, scalable aur reliable data exchange provide karta hai. ROS 2 mein DDS implementation ke through different nodes ke darmiyan communication hoti hai.

DDS ke through ROS 2 multiple DDS implementations support karta hai jaise ki:
- Fast DDS (RapidsDDS)
- Cyclone DDS
- RTI Connext DDS

### 4. Core ROS 2 Concepts

#### Nodes
Nodes ROS 2 programs ke individual processes hain. Har node ek specific task perform karta hai aur communication ke liye topics, services aur actions ka use karta hai.

#### Topics
Topics ROS 2 ke pub/sub communication model ke liye use kiye jate hain. Publishers messages send karte hain aur subscribers messages receive karte hain.

#### Services
Services request/response communication ke liye use kiye jate hain. Client request bhejta hai aur server response deta hai.

#### Actions
Actions long-running tasks ke liye use kiye jate hain jo feedback aur goal preemption support karte hain.

## Exercises and Questions

1. **Multiple Choice:** ROS 2 mein communication ke liye konsa protocol use hota hai?
   a) HTTP
   b) TCP/IP
   c) DDS
   d) UDP

2. **Short Answer:** ROS 1 aur ROS 2 ke darmiyan 3 main farq bataiye.

3. **Application:** Ek simple robot application design kariye jisme 3 nodes ho aur explain kariye ke ye nodes kaise communicate karenge.

4. **Analysis:** Kyun ROS 2 mein DDS ka use kiya gaya? DDS ke benefits kya hain?

5. **Implementation:** Ek simple publisher aur subscriber pair banaiye jo "Hello World" messages exchange kare.

6. **Conceptual:** Quality of Service (QoS) profiles ke bare mein explain kariye aur unke importance ko describe kariye.

7. **Comparison:** ROS 2 ke different DDS implementations ke darmiyan comparison kariye.

8. **Scenario:** Real-time robotics application mein ROS 2 ka use karte hue security considerations kya honge?

9. **Problem Solving:** Agar ek node messages receive nahi kar raha hai to troubleshooting kaise karenge?

10. **Research:** ROS 2 ke latest version aur upcoming features ke bare mein research kariye aur unka impact explain kariye.