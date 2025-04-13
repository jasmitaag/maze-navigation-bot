# **Maze-navigation-bot**
## **Hybrid A* + Wall-Following: Low-Compute Maze Navigation in ROS 2**

### **Overview**
```This project is my submission for the ROS Navigator challenge.Maze navigation is a foundational challenge in robotics that tests real-time planning, path execution, and local decision-making under uncertainty. This project was built with a focus on:
                   - Creating a reliable autonomous robot that solves mazes intelligently
                   - Exploring planner fusion — combining global and local strategies
                   - Keeping computation lightweight for real-time performance```
```

### **Approach**
The robot starts by following a smooth path generated by Hybrid A*. A central node monitors progress in real-time. If the path gets blocked or fails, the system automatically switches to wall-following to keep moving through the maze. Once a clear path is found again, it resumes Hybrid A*, ensuring both intelligence and resilience during navigation.


### **Key features**
```
    -#### **Hybrid Navigation Strategy**
      Seamlessly combines Hybrid A* for optimal global planning and a custom Dynamic Wall-Following fallback for robust local obstacle handling.
    -#### **Real-Time Planner Switching**
      Dynamically switches between planning modes based on sensor feedback — no human intervention needed, ensuring smooth and adaptive navigation.
    -#### **Efficient Maze Traversal**
      Handles narrow corridors, sharp turns, and unexpected blockages with ease using intelligent wall-following behavior.
    -#### **Low Compute Load**
      Designed to run efficiently on low-resource hardware (like Raspberry Pi-class systems), making it ideal for real-world deployment on compact robots.
    -#### **Goal-Oriented Recovery**
      Even when obstacles disrupt the path, the system stays focused on reaching the goal rather than wandering endlessly.
    -#### **Simulated & Ready to Deploy**
      Tested in Gazebo simulation for accuracy and consistency — bridging the gap between virtual tests and real-world application.
```
### **Tech Stack**
```
Language: Python3, C++
Framework: ROS 2
Simulation: Gazebo
Robot Model: TurtleBot3 (Burger)
Planning: Hybrid A*, custom DWF
```








