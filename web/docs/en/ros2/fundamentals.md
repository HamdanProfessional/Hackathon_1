---
id: ros2-fundamentals
title: ROS 2 Fundamentals
sidebar_position: 1
---

# ROS 2: The Robotic Nervous System

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is the industry-standard middleware for building robotic applications. Think of it as the nervous system that coordinates all parts of a robot—sensors, actuators, decision-making modules, and control systems.

Unlike ROS 1, ROS 2 is designed for:
- **Real-time performance**: Critical for safety in physical robots
- **Multi-robot systems**: Coordinate fleets of robots
- **Production environments**: Industrial-grade reliability
- **Security**: Encrypted communication between nodes

## Core Architecture

ROS 2 uses a **publish-subscribe** pattern where independent processes (called **nodes**) communicate through **topics**.

### Nodes

A **node** is a single process that performs a specific task. Examples:
- A camera driver node publishes image data
- A perception node subscribes to images and detects objects
- A control node subscribes to object locations and moves the robot

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

**Topics** are named buses over which nodes exchange messages. They use a publish-subscribe model:
- One or more nodes **publish** data to a topic
- One or more nodes **subscribe** to receive that data

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

### Services

For request-response patterns, ROS 2 provides **services**. Unlike topics (which are asynchronous), services are **synchronous**.

Use cases:
- Request robot to move to a specific position
- Query sensor calibration status
- Trigger a specific action

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

## URDF: Robot Modeling

**URDF** (Unified Robot Description Format) is an XML format for describing robot geometry, kinematics, and dynamics.

A simple robot arm in URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.5 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

## Building Your First ROS 2 Package

Create a Python package:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy std_msgs
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Key Commands

| Command | Purpose |
|---------|---------|
| `ros2 node list` | List all running nodes |
| `ros2 topic list` | List all active topics |
| `ros2 topic echo /topic_name` | Monitor topic data |
| `ros2 run package_name node_name` | Launch a node |
| `ros2 launch package_name launch_file.py` | Launch multiple nodes |

## Next Steps

In the next chapter, we'll explore how to integrate Python-based AI agents with ROS 2 controllers using `rclpy`, enabling your robots to make intelligent decisions in real-time.

:::tip Personalization
Hardware-specific installation instructions are available! Click "Personalize" to see ROS 2 setup steps for your specific configuration (RTX4090, Jetson, Laptop, or Cloud).
:::

---

**Next Chapter**: [Simulation Fundamentals →](/docs/en/simulation/gazebo-basics)
