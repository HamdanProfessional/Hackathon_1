---
id: isaac-sim-getting-started
title: NVIDIA Isaac Sim شروعات
sidebar_position: 1
---

# AI-Robot دماغ: NVIDIA Isaac

## Isaac Sim کیا ہے؟

**NVIDIA Isaac Sim** NVIDIA Omniverse پر بنایا گیا ایک photorealistic روبوٹکس simulator ہے۔ یہ فراہم کرتا ہے:

- **Photorealistic rendering**: Synthetic data generation کے لیے
- **Physics-based simulation**: درست contact dynamics
- **Domain randomization**: مضبوط AI models train کریں
- **Sim-to-real transfer**: جسمانی روبوٹس پر models deploy کریں

## سسٹم کی ضروریات

### کم سے کم
- **GPU**: NVIDIA RTX 3070 (8GB VRAM)
- **RAM**: 32GB
- **OS**: Ubuntu 22.04 یا Windows 10/11

### تجویز کردہ
- **GPU**: NVIDIA RTX 4090 (24GB VRAM)
- **RAM**: 64GB DDR5

## پہلا Scene

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

world = World()
cube = DynamicCuboid(
    prim_path="/World/Cube",
    position=[0, 0, 1.0],
    size=0.5,
    color=[1.0, 0.0, 0.0]
)

world.reset()
for _ in range(1000):
    world.step(render=True)

simulation_app.close()
```

:::tip Personalization
ہارڈویئر کی ضروریات نمایاں طور پر مختلف ہوتی ہیں۔ اپنے ہارڈویئر پروفائل (RTX4090، Jetson، Laptop، یا Cloud) کے لیے تجویز کردہ Isaac Sim settings دیکھنے کے لیے "Personalize" کلک کریں۔
:::

---

**اگلا Chapter**: [Vision-Language-Action Models →](/docs/ur/vla/introduction)
