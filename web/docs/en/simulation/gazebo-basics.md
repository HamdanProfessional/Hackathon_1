---
id: gazebo-basics
title: Gazebo Simulation Basics
sidebar_position: 1
---

# The Digital Twin: Gazebo Simulation

## Why Simulation?

Before deploying code to expensive physical robots, we need a safe environment to test, iterate, and validate our algorithms. **Gazebo** is an open-source robotics simulator that provides:

- **Physics simulation**: Gravity, collisions, friction, inertia
- **Sensor simulation**: Cameras, LiDAR, IMUs, force/torque sensors
- **ROS 2 integration**: Native support for ros2_control
- **Cost-effective testing**: No hardware damage from failed experiments

## Gazebo Architecture

Gazebo consists of several components:

1. **Gazebo Server (gzserver)**: Runs physics simulation and sensor generation
2. **Gazebo Client (gzclient)**: Provides 3D visualization
3. **Plugins**: Extend functionality (sensors, actuators, world features)

## SDF: Simulation Description Format

While URDF describes robot kinematics, **SDF** (Simulation Description Format) is Gazebo's native format for complete simulation worlds.

Example robot in SDF:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="mobile_robot">
    <link name="chassis">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.166</ixx>
          <iyy>0.416</iyy>
          <izz>0.416</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.0 0.0 1.0 1</ambient>
          <diffuse>0.0 0.0 1.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

## Launching a Simulation

Basic Gazebo launch with ROS 2:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'worlds/empty.world'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', 'model.sdf'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
```

## Physics Configuration

Gazebo supports multiple physics engines. Configure physics parameters:

```xml
<world name="robot_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>

  <gravity>0 0 -9.81</gravity>
</world>
```

## Simulating Sensors

### Camera

```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
</sensor>
```

### LiDAR

```xml
<sensor name="laser" type="ray">
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>30.0</max>
    </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so"/>
</sensor>
```

## Debugging Tips

| Issue | Solution |
|-------|----------|
| Robot falls through ground | Check collision geometry matches visual |
| Unstable physics | Reduce `max_step_size` (e.g., 0.001s) |
| Slow simulation | Reduce sensor update rates |
| Robot doesn't spawn | Verify SDF syntax with `gz sdf -k model.sdf` |

## Integration with ROS 2 Control

Gazebo interfaces with `ros2_control` for hardware abstraction:

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

This allows the same control code to work in both simulation and real hardware.

:::tip Hardware-Specific Performance
Simulation performance varies greatly by hardware. Click "Personalize" to see recommended Gazebo settings for your GPU (RTX4090, Jetson, Laptop, or Cloud).
:::

---

**Next Chapter**: [Isaac Sim Introduction →](/docs/en/isaac-sim/getting-started)
