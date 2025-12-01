# Physical AI & Humanoid Robotics: Course Syllabus & Source Material

**Source of Truth:** This file contains the core curriculum, module structures, and hardware requirements for the textbook content generation.

---

## 1. Course Overview

**Focus:** AI Systems in the Physical World. Embodied Intelligence.
**Goal:** Bridge the digital brain and the physical body. Students apply AI knowledge to control Humanoid Robots in simulated and real-world environments.
**Context:** The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces Physical AI—AI systems that function in reality and comprehend physical laws.

### Learning Outcomes
- Understand Physical AI principles and embodied intelligence.
- Master ROS 2 (Robot Operating System) for robotic control.
- Simulate robots with Gazebo and Unity.
- Develop with NVIDIA Isaac AI robot platform.
- Design humanoid robots for natural interactions.
- Integrate GPT models for conversational robotics.

---

## 2. Module Breakdown

### Module 1: The Robotic Nervous System (ROS 2)
**Theme:** Middleware for robot control.
- **Key Concepts:** ROS 2 Nodes, Topics, Services, Actions.
- **Integration:** Bridging Python Agents to ROS controllers using `rclpy`.
- **Modeling:** Understanding URDF (Unified Robot Description Format) for humanoids.

### Module 2: The Digital Twin (Gazebo & Unity)
**Theme:** Physics simulation and environment building.
- **Gazebo:** Simulating physics (rigid body dynamics), gravity, and collisions.
- **Unity:** High-fidelity rendering and human-robot interaction.
- **Sensors:** Simulating LiDAR, Depth Cameras, and IMUs.

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
**Theme:** Advanced perception and training.
- **Isaac Sim:** Photorealistic simulation and synthetic data generation.
- **Isaac ROS:** Hardware-accelerated VSLAM (Visual SLAM) and navigation.
- **Nav2:** Path planning for bipedal humanoid movement.

### Module 4: Vision-Language-Action (VLA)
**Theme:** The convergence of LLMs and Robotics.
- **Voice-to-Action:** Using OpenAI Whisper for voice commands.
- **Cognitive Planning:** Using LLMs to translate natural language (e.g., "Clean the room") into a sequence of ROS 2 actions.
- **Capstone Project:** The Autonomous Humanoid. A simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

---

## 3. Weekly Schedule

| Week | Topic | Details |
| :--- | :--- | :--- |
| **1-2** | **Intro to Physical AI** | Foundations of embodied intelligence. Overview of humanoids. Sensors: LIDAR, cameras, IMUs, force/torque. |
| **3-5** | **ROS 2 Fundamentals** | Architecture, Nodes, Topics, Services. Building packages with Python. Launch files & parameters. |
| **6-7** | **Robot Simulation** | Gazebo setup. URDF/SDF formats. Physics & Sensor simulation. Intro to Unity for visualization. |
| **8-10** | **NVIDIA Isaac Platform** | Isaac SDK & Sim. AI perception/manipulation. Reinforcement learning. Sim-to-real transfer. |
| **11-12** | **Humanoid Development** | Kinematics & Dynamics. Bipedal locomotion/balance. Manipulation/grasping. Natural HRI design. |
| **13** | **Conversational Robotics** | Integrating GPT models. Speech recognition (Whisper). Multi-modal interaction (Speech, Gesture, Vision). |

---

## 4. Hardware Requirements

This course sits at the intersection of Physics Simulation, Visual Perception, and Generative AI.

### A. The "Digital Twin" Workstation (Required)
*Essential for running NVIDIA Isaac Sim & VLA models.*
- **GPU:** NVIDIA RTX 4070 Ti (12GB VRAM) min. Ideal: RTX 3090/4090 (24GB).
- **CPU:** Intel Core i7 (13th Gen+) or AMD Ryzen 9.
- **RAM:** 64 GB DDR5 (32 GB min).
- **OS:** Ubuntu 22.04 LTS (ROS 2 Humble/Iron is native to Linux).

### B. The "Physical AI" Edge Kit (The Brain)
*Deploys code to a real embedded system.*
- **Computer:** NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB). (~$249 for Super Dev Kit).
- **Vision:** Intel RealSense D435i (includes IMU). (~$349).
- **Voice:** ReSpeaker USB Mic Array v2.0. (~$69).
- **Storage:** High-endurance 128GB microSD.

### C. Robot Lab Options
1.  **Proxy Approach (Budget):** Unitree Go2 Edu (Quadruped) or Robotic Arm. ~$1,800.
2.  **Miniature Humanoid:** Unitree G1 (~$16k) or Robotis OP3 (~$12k). *Budget:* Hiwonder TonyPi Pro (~$600 - limited AI capability).
3.  **Premium (Sim-to-Real):** Unitree G1 Humanoid. Full bipedal dynamics.

### D. Cloud Alternative ("The Ether Lab")
*For students without RTX workstations.*
- **Cloud:** AWS g5.2xlarge (A10G GPU) or g6e.xlarge.
- **Cost:** ~$205/quarter (approx 10 hrs/week).
- **Latency Trap:** You can Train in cloud, but must Inference on local Edge Kit to avoid control latency.

---

## 5. Assessments
1.  **ROS 2 Package:** Development project.
2.  **Simulation:** Gazebo implementation.
3.  **Perception:** Isaac-based pipeline.
4.  **Capstone:** Simulated humanoid robot with conversational AI (Voice -> Plan -> Action).