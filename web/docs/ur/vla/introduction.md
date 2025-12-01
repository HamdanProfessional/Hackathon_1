---
id: vla-introduction
title: Vision-Language-Action ماڈلز
sidebar_position: 1
---

# VLA: LLMs اور روبوٹکس کا سنگم

## VLA ماڈلز کیا ہیں؟

**Vision-Language-Action (VLA)** ماڈلز روبوٹک AI کی cutting edge کی نمائندگی کرتے ہیں۔ وہ یکجا کرتے ہیں:

- **Vision**: بصری دنیا کو سمجھنا
- **Language**: قدرتی زبان کی سمجھ اور پیداوار (LLMs)
- **Action**: روبوٹک کنٹرول اور manipulation

## Voice-to-Action پائپ لائن

```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("command.mp3")
print(result["text"])
```

## LLMs کے ساتھ Planning

```python
from openai import AsyncOpenAI
import os

client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

async def plan_task(command: str):
    response = await client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a robot task planner."},
            {"role": "user", "content": command}
        ]
    )
    return response.choices[0].message.content
```

## Object Detection

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")

def detect_objects(frame):
    results = model(frame)
    return results
```

:::tip ہارڈویئر کی تحفظات
VLA ماڈلز LLM inference کے لیے نمایاں compute کی ضرورت ہوتی ہے۔ اپنے ہارڈویئر کے لیے تجویز کردہ deployment strategies دیکھنے کے لیے "Personalize" کلک کریں۔
:::

---

**مبارک ہو!** آپ نے Physical AI & Humanoid Robotics textbook کی بنیادی باتیں مکمل کر لی ہیں۔
