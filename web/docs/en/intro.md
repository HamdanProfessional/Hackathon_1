---
id: intro
title: Introduction to Physical AI & Humanoid Robotics
sidebar_position: 1
---

import PersonalizePrompt from '@site/src/components/PersonalizePrompt';

# Welcome to Physical AI & Humanoid Robotics

<PersonalizePrompt />

## What is Physical AI?

The future of artificial intelligence extends far beyond digital spaces—it's moving into the **physical world**. Physical AI represents AI systems that don't just process information but interact with reality, understand physical laws, and control robotic systems in real environments.

This course introduces **embodied intelligence**: AI that bridges the gap between the digital brain and the physical body.

## Course Focus

**Goal**: Master the tools and techniques to build AI-driven humanoid robots that can:
- Navigate complex environments
- Understand natural language commands
- Manipulate objects with precision
- Interact naturally with humans

**Approach**: We combine three critical domains:
1. **ROS 2** - The nervous system for robot control
2. **Simulation** - Digital twins for safe experimentation
3. **NVIDIA Isaac** - Advanced perception and AI training
4. **Vision-Language-Action (VLA)** - Conversational robotics

## Learning Outcomes

By the end of this course, you will be able to:

- ✅ Understand the principles of embodied intelligence
- ✅ Build and control robots using ROS 2 (Robot Operating System)
- ✅ Simulate complex robotic scenarios in Gazebo and Isaac Sim
- ✅ Develop AI perception pipelines using NVIDIA Isaac platform
- ✅ Design humanoid robots capable of natural human interaction
- ✅ Integrate large language models (LLMs) for conversational robotics

## Who This Course Is For

This course is designed for students and developers who want to work at the intersection of:
- **Artificial Intelligence**: Machine learning, computer vision, natural language processing
- **Robotics**: Embodied systems, control theory, kinematics
- **Simulation**: Physics engines, digital twins, synthetic data generation

## Hardware Considerations

Physical AI development requires specialized hardware depending on your goals:

### For Simulation & Development
- **GPU**: NVIDIA RTX 4070 Ti or higher (12GB+ VRAM)
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **RAM**: 32-64 GB
- **OS**: Ubuntu 22.04 LTS (native ROS 2 support)

### For Edge Deployment
- **Computer**: NVIDIA Jetson Orin Nano/NX
- **Vision**: Intel RealSense D435i depth camera
- **Voice**: ReSpeaker USB Mic Array

### Cloud Alternative
If you don't have access to high-end hardware, cloud platforms like AWS g5.2xlarge (A10G GPU) provide viable alternatives for training and simulation.

:::tip Hardware Profiles
This textbook adapts content based on your hardware setup. Click the "Personalize" button on any chapter to see instructions tailored to your specific configuration (RTX4090, Jetson, Laptop, or Cloud).
:::

## Course Structure

The course is divided into four interconnected modules:

1. **Module 1: The Robotic Nervous System (ROS 2)**
   - Master the middleware that controls robots
   - Learn about Nodes, Topics, Services, and Actions
   - Understand URDF robot modeling

2. **Module 2: The Digital Twin (Simulation)**
   - Build physics simulations with Gazebo
   - Create high-fidelity environments in Unity
   - Simulate sensors (LiDAR, cameras, IMUs)

3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
   - Advanced perception with Isaac Sim
   - Hardware-accelerated VSLAM and navigation
   - Reinforcement learning for manipulation

4. **Module 4: Vision-Language-Action (VLA)**
   - Voice-to-action pipelines with OpenAI Whisper
   - Cognitive planning using large language models
   - Capstone: Build an autonomous humanoid

## Getting Started

Ready to begin? Start with Module 1 to learn the foundational concepts of ROS 2, the nervous system that powers modern robotics.

:::info Prerequisites
Familiarity with Python programming is recommended. Experience with Linux command line is helpful but not required.
:::

---

**Next Chapter**: [ROS 2 Fundamentals →](/docs/en/ros2/fundamentals)
