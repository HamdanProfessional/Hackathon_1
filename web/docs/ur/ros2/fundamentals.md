---
id: ros2-fundamentals
title: ROS 2 بنیادی باتیں
sidebar_position: 1
---

# ROS 2: روبوٹک عصبی نظام

## ROS 2 کیا ہے؟

**ROS 2** (Robot Operating System 2) روبوٹک ایپلیکیشنز بنانے کے لیے industry-standard middleware ہے۔ اسے روبوٹ کے عصبی نظام کے طور پر سوچیں جو روبوٹ کے تمام حصوں کو coordinate کرتا ہے—sensors، actuators، فیصلہ سازی کے modules، اور کنٹرول سسٹمز۔

ROS 1 کے برعکس، ROS 2 کے لیے ڈیزائن کیا گیا ہے:
- **Real-time کارکردگی**: جسمانی روبوٹس میں حفاظت کے لیے اہم
- **Multi-robot سسٹمز**: روبوٹس کے بیڑے coordinate کریں
- **Production ماحول**: صنعتی درجے کی reliability
- **سیکیورٹی**: nodes کے درمیان encrypted communication

## بنیادی Architecture

ROS 2 ایک **publish-subscribe** pattern استعمال کرتا ہے جہاں آزاد processes (جنہیں **nodes** کہا جاتا ہے) **topics** کے ذریعے communicate کرتے ہیں۔

### Nodes

ایک **node** ایک واحد process ہے جو ایک مخصوص کام انجام دیتا ہے۔ مثالیں:
- ایک camera driver node تصویری ڈیٹا publish کرتا ہے
- ایک perception node تصاویر کو subscribe کرتا ہے اور اشیاء کا پتہ لگاتا ہے
- ایک control node object locations کو subscribe کرتا ہے اور روبوٹ کو حرکت دیتا ہے

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

**Topics** نام والی buses ہیں جن پر nodes پیغامات کا تبادلہ کرتے ہیں۔ وہ publish-subscribe ماڈل استعمال کرتے ہیں:
- ایک یا زیادہ nodes ایک topic پر ڈیٹا **publish** کرتے ہیں
- ایک یا زیادہ nodes وہ ڈیٹا وصول کرنے کے لیے **subscribe** کرتے ہیں

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

Request-response patterns کے لیے، ROS 2 **services** فراہم کرتا ہے۔ Topics (جو asynchronous ہیں) کے برعکس، services **synchronous** ہیں۔

Use cases:
- روبوٹ کو ایک مخصوص پوزیشن پر جانے کی request کریں
- Sensor calibration status query کریں
- ایک مخصوص action trigger کریں

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

**URDF** (Unified Robot Description Format) روبوٹ کی geometry، kinematics، اور dynamics کی وضاحت کے لیے ایک XML format ہے۔

URDF میں ایک سادہ robot arm:

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

## اپنا پہلا ROS 2 Package بنانا

Python package بنائیں:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy std_msgs
```

Workspace بنائیں:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## اہم Commands

| Command | مقصد |
|---------|------|
| `ros2 node list` | تمام چلتے ہوئے nodes list کریں |
| `ros2 topic list` | تمام active topics list کریں |
| `ros2 topic echo /topic_name` | Topic data monitor کریں |
| `ros2 run package_name node_name` | Node launch کریں |
| `ros2 launch package_name launch_file.py` | متعدد nodes launch کریں |

## اگلے قدم

اگلے chapter میں، ہم `rclpy` استعمال کرتے ہوئے Python-based AI agents کو ROS 2 controllers کے ساتھ integrate کرنے کا طریقہ دیکھیں گے، جو آپ کے روبوٹس کو real-time میں ذہین فیصلے کرنے کے قابل بنائے گا۔

:::tip Personalization
ہارڈویئر کی مخصوص installation ہدایات دستیاب ہیں! اپنی مخصوص تشکیل (RTX4090، Jetson، Laptop، یا Cloud) کے لیے ROS 2 سیٹ اپ کے قدم دیکھنے کے لیے "Personalize" کلک کریں۔
:::

---

**اگلا Chapter**: [Simulation Fundamentals →](/docs/ur/simulation/gazebo-basics)
