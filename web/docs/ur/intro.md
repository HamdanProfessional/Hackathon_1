---
id: intro
title: فزیکل AI اور ہیومنائیڈ روبوٹکس کا تعارف
sidebar_position: 1
---

# فزیکل AI اور ہیومنائیڈ روبوٹکس میں خوش آمدید

## فزیکل AI کیا ہے؟

مصنوعی ذہانت کا مستقبل صرف ڈیجیٹل دنیا تک محدود نہیں—یہ **حقیقی دنیا** میں داخل ہو رہی ہے۔ فزیکل AI ایسے AI سسٹمز کی نمائندگی کرتا ہے جو صرف معلومات پر کارروائی نہیں کرتے بلکہ حقیقت کے ساتھ تعامل کرتے ہیں، فزیکل قوانین کو سمجھتے ہیں، اور حقیقی ماحول میں روبوٹک سسٹمز کو کنٹرول کرتے ہیں۔

یہ کورس **embodied intelligence** متعارف کراتا ہے: AI جو ڈیجیٹل دماغ اور جسمانی جسم کے درمیان فاصلے کو ختم کرتا ہے۔

## کورس کا فوکس

**ہدف**: AI سے چلنے والے ہیومنائیڈ روبوٹس بنانے کے لیے ٹولز اور تکنیک میں مہارت حاصل کریں جو:
- پیچیدہ ماحول میں نیویگیٹ کر سکیں
- قدرتی زبان کے احکامات کو سمجھ سکیں
- اشیاء کو درستگی سے ہینڈل کر سکیں
- انسانوں کے ساتھ قدرتی طور پر تعامل کر سکیں

**نقطہ نظر**: ہم تین اہم شعبوں کو یکجا کرتے ہیں:
1. **ROS 2** - روبوٹ کنٹرول کے لیے عصبی نظام
2. **Simulation** - محفوظ تجربات کے لیے ڈیجیٹل ٹوئنز
3. **NVIDIA Isaac** - جدید ادراک اور AI تربیت
4. **Vision-Language-Action (VLA)** - گفتگو کرنے والی روبوٹکس

## سیکھنے کے نتائج

اس کورس کے اختتام تک، آپ قابل ہوں گے:

- ✅ embodied intelligence کے اصولوں کو سمجھنا
- ✅ ROS 2 (Robot Operating System) استعمال کرتے ہوئے روبوٹس بنانا اور کنٹرول کرنا
- ✅ Gazebo اور Isaac Sim میں پیچیدہ روبوٹک منظرنامے simulate کرنا
- ✅ NVIDIA Isaac پلیٹ فارم استعمال کرتے ہوئے AI perception pipelines تیار کرنا
- ✅ قدرتی انسانی تعامل کے قابل ہیومنائیڈ روبوٹس ڈیزائن کرنا
- ✅ گفتگو کرنے والی روبوٹکس کے لیے large language models (LLMs) کو integrate کرنا

## یہ کورس کس کے لیے ہے

یہ کورس ان طلباء اور ڈویلپرز کے لیے ڈیزائن کیا گیا ہے جو ان شعبوں کے سنگم پر کام کرنا چاہتے ہیں:
- **مصنوعی ذہانت**: machine learning، computer vision، natural language processing
- **روبوٹکس**: embodied systems، control theory، kinematics
- **Simulation**: physics engines، digital twins، synthetic data generation

## ہارڈویئر کی ضروریات

فزیکل AI ڈیولپمنٹ آپ کے اہداف کے لحاظ سے خصوصی ہارڈویئر کی ضرورت ہوتی ہے:

### Simulation اور Development کے لیے
- **GPU**: NVIDIA RTX 4070 Ti یا اس سے اوپر (12GB+ VRAM)
- **CPU**: Intel Core i7 (13th Gen+) یا AMD Ryzen 9
- **RAM**: 32-64 GB
- **OS**: Ubuntu 22.04 LTS (native ROS 2 سپورٹ)

### Edge Deployment کے لیے
- **Computer**: NVIDIA Jetson Orin Nano/NX
- **Vision**: Intel RealSense D435i depth camera
- **Voice**: ReSpeaker USB Mic Array

### Cloud متبادل
اگر آپ کے پاس high-end ہارڈویئر تک رسائی نہیں ہے، تو AWS g5.2xlarge (A10G GPU) جیسے cloud پلیٹ فارمز training اور simulation کے لیے قابل عمل متبادل فراہم کرتے ہیں۔

:::tip ہارڈویئر پروفائلز
یہ textbook آپ کے ہارڈویئر سیٹ اپ کی بنیاد پر مواد کو اپناتی ہے۔ اپنی مخصوص تشکیل (RTX4090، Jetson، Laptop، یا Cloud) کے مطابق ہدایات دیکھنے کے لیے کسی بھی chapter پر "Personalize" بٹن کلک کریں۔
:::

## کورس کی ساخت

کورس چار آپس میں جڑے ہوئے modules میں تقسیم ہے:

1. **Module 1: The Robotic Nervous System (ROS 2)**
   - روبوٹس کو کنٹرول کرنے والے middleware میں مہارت حاصل کریں
   - Nodes، Topics، Services، اور Actions کے بارے میں جانیں
   - URDF robot modeling کو سمجھیں

2. **Module 2: The Digital Twin (Simulation)**
   - Gazebo کے ساتھ physics simulations بنائیں
   - Unity میں high-fidelity environments بنائیں
   - Sensors (LiDAR، cameras، IMUs) simulate کریں

3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
   - Isaac Sim کے ساتھ جدید ادراک
   - Hardware-accelerated VSLAM اور navigation
   - Manipulation کے لیے reinforcement learning

4. **Module 4: Vision-Language-Action (VLA)**
   - OpenAI Whisper کے ساتھ voice-to-action pipelines
   - Large language models استعمال کرتے ہوئے cognitive planning
   - Capstone: ایک خودکار ہیومنائیڈ بنائیں

## شروعات کریں

شروع کرنے کے لیے تیار ہیں؟ ROS 2 کے بنیادی تصورات سیکھنے کے لیے Module 1 سے شروع کریں، وہ عصبی نظام جو جدید روبوٹکس کو طاقت دیتا ہے۔

:::info پیشگی ضروریات
Python programming سے واقفیت تجویز کی جاتی ہے۔ Linux command line کا تجربہ مددگار ہے لیکن ضروری نہیں۔
:::

---

**اگلا Chapter**: [ROS 2 Fundamentals →](/docs/ur/ros2/fundamentals)
