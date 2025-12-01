---
id: isaac-sim-getting-started
title: Getting Started with NVIDIA Isaac Sim
sidebar_position: 1
---

# The AI-Robot Brain: NVIDIA Isaac

## What is Isaac Sim?

**NVIDIA Isaac Sim** is a photorealistic robotics simulator built on NVIDIA Omniverse. It provides:

- **Photorealistic rendering**: Ray-traced graphics for synthetic data generation
- **Physics-based simulation**: Accurate contact dynamics and material properties
- **Domain randomization**: Train robust AI models with varied environments
- **Sim-to-real transfer**: Deploy trained models directly to physical robots
- **ROS 2 integration**: Seamless compatibility with ROS 2 workflows

Isaac Sim is essential for training perception and manipulation models that transfer to real-world robots.

## System Requirements

Isaac Sim demands significant compute resources:

### Minimum Requirements
- **GPU**: NVIDIA RTX 3070 (8GB VRAM)
- **CPU**: Intel i7 or AMD Ryzen 7
- **RAM**: 32GB
- **Storage**: 50GB SSD space
- **OS**: Ubuntu 22.04 or Windows 10/11

### Recommended
- **GPU**: NVIDIA RTX 4090 (24GB VRAM)
- **CPU**: Intel i9-13900K or AMD Ryzen 9 7950X
- **RAM**: 64GB DDR5
- **Storage**: NVMe SSD

:::warning Cloud Alternative
If you don't have an RTX GPU, use AWS g5.2xlarge (NVIDIA A10G) or g6e instances. Isaac Sim is pre-installed on NVIDIA NGC containers.
:::

## Installation

### Option 1: Omniverse Launcher (Recommended)

1. Download [NVIDIA Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. Install Isaac Sim from the Exchange tab
3. Launch Isaac Sim 2023.1.1 or later

### Option 2: Docker (Linux)

```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Option 3: Native Python (Advanced)

```bash
pip install isaacsim isaacsim-extscache-physics isaacsim-extscache-kit-sdk
```

## Your First Isaac Sim Scene

Launch Isaac Sim and create a basic scene with Python:

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()

# Add a cube
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="my_cube",
    position=[0, 0, 1.0],
    size=0.5,
    color=[1.0, 0.0, 0.0]  # Red
)

# Run simulation
world.reset()
for _ in range(1000):
    world.step(render=True)

simulation_app.close()
```

## Key Concepts

### Prims and USD

Isaac Sim uses **Universal Scene Description (USD)** to represent scenes. Everything is a **Prim** (primitive):

- `/World`: Root of the scene hierarchy
- `/World/Ground`: Ground plane
- `/World/Cube`: Your dynamic cube
- `/World/Robot`: Your robot model

### Physics

Isaac Sim uses **PhysX** for realistic physics simulation:

```python
from omni.isaac.core.prims import RigidPrim

rigid_object = RigidPrim(
    prim_path="/World/Object",
    mass=1.0,
    linear_velocity=[1.0, 0.0, 0.0]
)
```

### Actions and Observations

Reinforcement learning workflow:

```python
from omni.isaac.core.tasks import BaseTask

class PickAndPlaceTask(BaseTask):
    def get_observations(self):
        # Return robot state (positions, velocities)
        return {
            "joint_positions": self.robot.get_joint_positions(),
            "object_position": self.target_object.get_world_pose()[0]
        }

    def calculate_reward(self):
        # Compute reward based on task completion
        distance = np.linalg.norm(gripper_pos - object_pos)
        return -distance  # Negative distance as reward
```

## Synthetic Data Generation

Isaac Sim excels at generating labeled training data:

```python
from omni.replicator.core import BasicWriter

writer = BasicWriter(
    output_dir="output/data",
    semantic_types=["class"],
    rgb=True,
    bounding_box_2d_tight=True
)

# Domain randomization
import omni.replicator.core as rep

with rep.new_layer():
    # Randomize lighting
    sphere_light = rep.create.light(light_type="Sphere")
    with sphere_light:
        rep.modify.attribute("intensity", rep.distribution.uniform(1000, 5000))

    # Randomize camera position
    camera = rep.create.camera()
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, -5, 2), (5, 5, 4)),
            look_at="/World/Robot"
        )
```

## ROS 2 Integration

Isaac Sim natively publishes to ROS 2 topics:

```python
from omni.isaac.ros2_bridge import ROS2Bridge

# Create ROS 2 bridge
ros2_bridge = ROS2Bridge()

# Publish camera images
ros2_bridge.create_camera_helper(
    camera_prim_path="/World/Camera",
    topic_name="/camera/rgb/image_raw",
    frame_id="camera_frame"
)

# Publish joint states
ros2_bridge.create_joint_state_publisher(
    robot_prim_path="/World/Robot"
)
```

## Next Steps

In the next chapters, we'll explore:
- Training manipulation policies with Isaac Gym
- Sim-to-real transfer techniques
- Visual SLAM with Isaac ROS

:::tip Personalization
Hardware requirements vary significantly. Click "Personalize" to see recommended Isaac Sim settings and alternatives for your hardware profile (RTX4090, Jetson, Laptop, or Cloud).
:::

---

**Next Chapter**: [Vision-Language-Action Models →](/docs/en/vla/introduction)
