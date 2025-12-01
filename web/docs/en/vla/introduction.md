---
id: vla-introduction
title: Vision-Language-Action Models
sidebar_position: 1
---

# VLA: The Convergence of LLMs and Robotics

## What are Vision-Language-Action Models?

**Vision-Language-Action (VLA)** models represent the cutting edge of robotic AI. They combine:

- **Vision**: Understanding the visual world (cameras, depth sensors)
- **Language**: Natural language understanding and generation (LLMs)
- **Action**: Robotic control and manipulation

VLA models enable robots to understand commands like "Pick up the red cup and place it on the table" and translate them into precise motor actions.

## The VLA Pipeline

A complete VLA system consists of several stages:

```
User Voice Command
    ↓
Speech-to-Text (Whisper)
    ↓
Language Model (GPT-4)
    ↓
Task Planning (ROS 2 Actions)
    ↓
Vision Processing (Object Detection)
    ↓
Motion Planning (Nav2/MoveIt)
    ↓
Robot Execution
```

## Voice-to-Action with OpenAI Whisper

**Whisper** converts speech to text with high accuracy:

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("command.mp3")
print(result["text"])
# Output: "Pick up the red cup and place it on the table"
```

### Real-time Speech Recognition

For continuous speech recognition:

```python
import pyaudio
import numpy as np

# Audio stream
p = pyaudio.PyAudio()
stream = p.open(
    format=pyaudio.paInt16,
    channels=1,
    rate=16000,
    input=True,
    frames_per_buffer=1024
)

# Buffer audio
audio_buffer = []
while True:
    data = stream.read(1024)
    audio_buffer.append(np.frombuffer(data, dtype=np.int16))

    if len(audio_buffer) >= 16:  # ~1 second at 16kHz
        audio = np.concatenate(audio_buffer)
        result = model.transcribe(audio)
        command = result["text"]
        # Process command...
        audio_buffer = []
```

## Cognitive Planning with LLMs

Use GPT-4 to decompose high-level commands into ROS 2 action sequences:

```python
from openai import AsyncOpenAI
import os

client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

async def plan_task(command: str):
    response = await client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": """You are a robot task planner.
            Convert natural language commands into JSON action sequences.

            Available actions:
            - navigate(x, y): Move to coordinates
            - detect_object(class): Find object by class
            - grasp(): Close gripper
            - release(): Open gripper
            - raise_arm(height): Lift arm to height
            """},
            {"role": "user", "content": command}
        ]
    )

    return response.choices[0].message.content

# Example usage
command = "Pick up the red cup and place it on the table"
plan = await plan_task(command)
print(plan)
```

Output JSON:
```json
{
  "actions": [
    {"type": "detect_object", "params": {"class": "cup", "color": "red"}},
    {"type": "navigate", "params": {"x": "object_x", "y": "object_y"}},
    {"type": "grasp", "params": {}},
    {"type": "raise_arm", "params": {"height": 0.5}},
    {"type": "navigate", "params": {"x": "table_x", "y": "table_y"}},
    {"type": "release", "params": {}}
  ]
}
```

## Object Detection with Computer Vision

Integrate YOLO for real-time object detection:

```python
from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")

def detect_objects(frame):
    results = model(frame)
    detections = []

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            detections.append({
                "class": model.names[cls],
                "confidence": conf,
                "bbox": [x1, y1, x2, y2]
            })

    return detections
```

## Executing Actions with ROS 2

Translate planned actions to ROS 2 action clients:

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class RobotController:
    def __init__(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    async def execute_plan(self, plan):
        for action in plan["actions"]:
            if action["type"] == "navigate":
                await self.navigate(action["params"]["x"], action["params"]["y"])
            elif action["type"] == "grasp":
                await self.grasp()
            # ... other actions

    async def navigate(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        await self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        result = await goal_handle.get_result_async()
        return result.status
```

## Multi-Modal Interaction

Combine speech, gestures, and vision:

```python
class MultiModalInterface:
    def __init__(self):
        self.speech_recognizer = whisper.load_model("base")
        self.gesture_detector = load_gesture_model()
        self.vision_model = YOLO("yolov8n.pt")

    async def process_input(self, audio, video_frame):
        # Process speech
        speech_text = self.speech_recognizer.transcribe(audio)["text"]

        # Detect pointing gesture
        gesture = self.gesture_detector(video_frame)

        # If user is pointing, detect target object
        if gesture == "pointing":
            objects = self.vision_model(video_frame)
            target = self.get_pointed_object(gesture.direction, objects)
            command = f"{speech_text} the {target}"
        else:
            command = speech_text

        return command
```

## Capstone Project: Autonomous Humanoid

Build a complete VLA system:

1. **Input**: Voice command "Clean the room"
2. **Planning**: LLM generates subtasks (navigate, identify objects, pick up, dispose)
3. **Perception**: Vision system identifies trash objects
4. **Navigation**: Nav2 plans collision-free paths
5. **Manipulation**: MoveIt controls arm to grasp objects
6. **Verification**: Vision confirms task completion

:::tip Hardware Considerations
VLA models require significant compute for LLM inference. Click "Personalize" to see recommended deployment strategies for your hardware (RTX4090 edge inference, Jetson optimization, or cloud-based processing).
:::

---

**Congratulations!** You've completed the Physical AI & Humanoid Robotics textbook fundamentals. Continue exploring the advanced topics in each module.
