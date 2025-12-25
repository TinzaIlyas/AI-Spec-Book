---
sidebar_position: 3
---

# ROS 2 Actions aur Advanced Communication

## Chapter Title: ROS 2 Actions aur Advanced Communication
**Roman Urdu Version:** ROS 2 actions aur advanced communication

## Summary
Ye chapter ROS 2 ke advanced communication patterns ko cover karta hai, jisme actions, parameters, events aur complex communication scenarios shamil hain. Aap yahan se long-running tasks aur complex robot behaviors ke implementation ke bare mein seekhenge.

Actions ROS 2 ke powerful communication pattern hain jo long-running tasks ke liye suitable hote hain. Ye feedback aur goal preemption provide karte hain jo complex robot operations ke liye zaroori hote hain.

## Learning Objectives
- ROS 2 actions ke concepts aur implementation ko samajhna
- Parameters aur events ka use karna
- Complex communication scenarios design karna
- Action servers aur clients implement karna

## Key Topics
1. Actions ke Concepts aur Use Cases
2. Action Message Structure
3. Action Servers aur Clients
4. Parameters aur Configuration Management
5. Events aur Callbacks
6. Communication Patterns aur Best Practices
7. Debugging aur Monitoring

## Chapter Content

### 1. Actions ke Concepts aur Use Cases

Actions long-running tasks ke liye ROS 2 ka communication pattern hai jo following features provide karta hai:
- Feedback: Task execution ke dauran intermediate results
- Goal Preemption: Running tasks ko cancel ya modify karna
- Result: Task completion par final result

Actions typical use cases:
- Navigation tasks
- Manipulation operations
- Calibration procedures
- Data collection processes

### 2. Action Message Structure

Har action 3 different message types define karti hai:
- Goal: Task ke parameters
- Result: Task completion result
- Feedback: Intermediate progress updates

Ye messages .action files mein define kiye jate hain aur different languages ke liye generate kiye jate hain.

### 3. Action Servers aur Clients

Action server following responsibilities handle karta hai:
- Goals accept ya reject karna
- Goal execution perform karna
- Feedback send karna
- Results provide karna
- Goal preemption handle karna

Action client following operations perform karta hai:
- Goals send karna
- Goals cancel karna
- Feedback receive karna
- Results receive karna

### 4. Parameters aur Configuration Management

ROS 2 parameters dynamic configuration ke liye use kiye jate hain. Ye runtime par change kiye ja sakte hain aur nodes ke behavior ko control karte hain.

Parameter features:
- Hierarchical naming
- Type safety
- Automatic serialization
- Node-level aur global parameters

### 5. Events aur Callbacks

ROS 2 events communication lifecycle ke different stages ko monitor aur handle karne ke liye provide kiye jate hain:
- Subscription events
- Publisher events
- Service events
- Timer events

## Exercises and Questions

1. **Implementation:** Ek action server implement kariye jo robot navigation task perform karta ho aur intermediate feedback provide karta ho.

2. **Design:** Ek robot arm manipulation task ke liye action design kariye aur goal, result aur feedback structure define kariye.

3. **Analysis:** Actions aur services ke darmiyan when to use ka comparison kariye aur appropriate use cases provide kariye.

4. **Problem Solving:** Ek action client mein goal preemption ka handling kaise implement karenge?

5. **Coding:** Ek dynamic parameter system implement kariye jo robot behavior ko runtime par modify kar sakta ho.

6. **Scenario:** Multi-robot coordination mein actions ka use kaise kiya ja sakta hai? Ek practical example provide kariye.

7. **Conceptual:** Action lifecycle states aur transitions ko explain kariye aur unke importance ko describe kariye.

8. **Application:** Ek complex robot task (jaise mapping ya SLAM) ke liye action-based architecture design kariye.

9. **Debugging:** Action-based systems mein debugging aur monitoring kaise perform karenge?

10. **Research:** ROS 2 actions ke latest enhancements aur upcoming features ke bare mein research kariye.