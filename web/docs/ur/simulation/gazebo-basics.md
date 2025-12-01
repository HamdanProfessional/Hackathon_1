---
id: gazebo-basics
title: Gazebo Simulation بنیادی باتیں
sidebar_position: 1
---

# Digital Twin: Gazebo Simulation

## Simulation کیوں؟

مہنگے جسمانی روبوٹس پر کوڈ deploy کرنے سے پہلے، ہمیں اپنے algorithms کو test، iterate، اور validate کرنے کے لیے ایک محفوظ ماحول کی ضرورت ہے۔ **Gazebo** ایک open-source روبوٹکس simulator ہے جو فراہم کرتا ہے:

- **Physics simulation**: کشش ثقل، collisions، friction، inertia
- **Sensor simulation**: Cameras، LiDAR، IMUs، force/torque sensors
- **ROS 2 integration**: ros2_control کے لیے native سپورٹ
- **قیمت مؤثر testing**: ناکام تجربات سے ہارڈویئر کو کوئی نقصان نہیں

## SDF Format

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="mobile_robot">
    <link name="chassis">
      <inertial>
        <mass>10.0</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## Simulation Launch کرنا

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', 'model.sdf']
        )
    ])
```

:::tip ہارڈویئر مخصوص کارکردگی
Simulation کارکردگی ہارڈویئر کے لحاظ سے بہت مختلف ہوتی ہے۔ اپنے GPU (RTX4090، Jetson، Laptop، یا Cloud) کے لیے تجویز کردہ Gazebo settings دیکھنے کے لیے "Personalize" کلک کریں۔
:::

---

**اگلا Chapter**: [Isaac Sim Introduction →](/docs/ur/isaac-sim/getting-started)
