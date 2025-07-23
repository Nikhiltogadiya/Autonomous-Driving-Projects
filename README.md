# Autonomous-Driving-Projects

# Autonomous Driving Projects

Welcome to my repository for autonomous driving projects completed as part of the "Autonomous Driving SS 2024" course at Ostfalia University of Applied Sciences. This collection showcases practical implementations of key concepts in autonomous systems, including reinforcement learning, sensor fusion, and vehicle control.

## Table of Contents

- [Autonomous-Driving-Projects](#autonomous-driving-projects)
- [Autonomous Driving Projects](#autonomous-driving-projects-1)
  - [Table of Contents](#table-of-contents)
    - [**Task 1: Highway Driving with Proximal Policy Optimization (PPO)**](#task-1-highway-driving-with-proximal-policy-optimization-ppo)
    - [**Task 2: Environment Perception with Dempster-Shafer Grid Fusion**](#task-2-environment-perception-with-dempster-shafer-grid-fusion)
    - [**Task 3: Adaptive Cruise Control with the Intelligent Driver Model (IDM)**](#task-3-adaptive-cruise-control-with-the-intelligent-driver-model-idm)

---

### **Task 1: Highway Driving with Proximal Policy Optimization (PPO)**

This project focuses on training a reinforcement learning agent to drive autonomously on a highway. The primary goal is to teach the vehicle to avoid collisions, stay in the rightmost lane, and maintain a target speed. The agent is trained using the **Proximal Policy Optimization (PPO)** algorithm.

* **Objective**: Develop a policy-based RL agent for autonomous highway driving.
* **Algorithm**: Proximal Policy Optimization (PPO), an actor-critic method.
* **Key Concepts**: Reinforcement Learning, Policy Gradients, Actor-Critic Networks, Hyperparameter Tuning.
* **Environment**: `highway-env` for Gymnasium.

<!-- [![Highway Driving Simulation](https://i.imgur.com/GZ6XzJm.png)](task_1_PPO_Highway_Driving/) -->

> **[Go to Task 1 Details](./Task_1_PPO_Highway_Driving/)**

---

### **Task 2: Environment Perception with Dempster-Shafer Grid Fusion**

This task implements a sensor fusion pipeline to build an earth-fixed grid map of the vehicle's surroundings. Data from a LiDAR sensor is processed and fused using the **Dempster-Shafer theory** to represent the state of each grid cell as `occupied`, `free`, or `unknown`. This approach allows for robustly handling sensor uncertainty.

* **Objective**: Create an earth-fixed occupancy grid using LiDAR data.
* **Algorithm**: Dempster-Shafer Theory for evidence-based fusion.
* **Key Concepts**: Sensor Fusion, Occupancy Grids, Dempster-Shafer Theory, Coordinate Transformations, ROS2.
* **Environment**: Webots and ROS2.

<!-- [![Dempster-Shafer Grid](https://i.imgur.com/gK9q9L6.png)](Task_2_Dempster-Shafer_Grid_Fusion/) -->

> **[Go to Task 2 Details](./Task_2_Dempster-Shafer_Grid_Fusion/)**

---

### **Task 3: Adaptive Cruise Control with the Intelligent Driver Model (IDM)**

This project implements an Adaptive Cruise Control (ACC) system. The vehicle uses radar data to detect a lead vehicle and adjusts its speed based on the **Intelligent Driver Model (IDM)**. The goal is to maintain a safe following distance and adapt to the speed of the vehicle ahead without manual intervention.

* **Objective**: Implement a longitudinal controller for vehicle following.
* **Algorithm**: Intelligent Driver Model (IDM).
* **Key Concepts**: Adaptive Cruise Control (ACC), Longitudinal Vehicle Control, Sensor Data Processing (Radar), ROS2.
* **Environment**: Webots and ROS2.

<!-- [![ACC Simulation](https://i.imgur.com/qLhV8pQ.png)](Task_3_Intelligent_Driver_Model_ACC/) -->

> **[Go to Task 3 Details](./Task_3_Intelligent_Driver_Model_ACC/)**