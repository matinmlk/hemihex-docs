---
title: Offline Speech to Text (ASR)
sidebar_position: 0
---

# 1.Offline speech to text (ASR)

## 1. Introduction

### 1.1 What is "ASR"?

ASR (Automatic Speech Recognition) is a technology that converts human speech signals into text. It is widely used in intelligent assistants, voice command control, telephone customer service automation, and real-time subtitle generation. The goal of ASR is to enable machines to "understand" human speech and convert it into a form that computers can process and understand.

### 1.2 Implementation Principles

The implementation of an ASR system relies primarily on the following key technical components:

#### 1. Acoustic Model

#### 2. Language Model

#### 3. Pronunciation Dictionary

#### 4. Decoder

#### 5. End-to-End ASR

In general, modern ASR systems achieve efficient and accurate speech-to-text conversion by combining the aforementioned components and leveraging large datasets and powerful computing resources for training. With technological advances, the performance of ASR systems continues to improve, and their application scenarios are becoming increasingly broad.

--

## 2. Code Analysis

### Key Code

#### 1. Speech Processing and Recognition Core ( largemodel/largemodel/asr.py )

[TODO]
### Code Analysis

ASR (speech-to-text) functionality is provided by the ASRNode node ( asr.py ). This node is responsible for recording, converting, and publishing audio.

## 3. Practical Operations

### 3.1 Configuring Offline ASR

To enable offline ASR, you need to correctly configure the HemiHex.yaml file and ensure that the local model is correctly placed.

Open the configuration file :

```python

vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

```python

```

```python
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/confirm the following key configuration :

```python

asr
:
#Voice node parameters
ros__parameters
:
# ...
use_oline_asr
:
False
# KEY: Must be set to False to enable offline ASR
mic_serial_port
:
"/dev/ttyUSB0"
# Microphone serial port alias
mic_index
:
0
# Microphone Device Index
language
:
'en'
# asr language, 'zh' or 'en'
regional_setting
:
"international"
```

```python

```

```python
asr
:
#Voice node parameters
```

```python
ros__parameters
:
```

```python
# ...
```

```python
use_oline_asr
:
False
# KEY: Must be set to False to enable offline ASR
```

```python
mic_serial_port
:
"/dev/ttyUSB0"
# Microphone serial port alias
```

```python
mic_index
:
0
# Microphone Device Index
```

```python
language
:
'en'
# asr language, 'zh' or 'en'
```

```python
regional_setting
:
"international"
```

Make sure use_oline_asr is set to False to use the local model.

In the terminal, enter ls /dev/ttyUSB* to check if the USB device number assigned to the voice module is USB0. If not, replace the 0 in the configuration file with your desired device number.

Select "zh" for Chinese and "en" for English.

Also, specify the path to the offline model in large_model_interface.yaml .

Open the file in the terminal:

```python

vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```python

```

```python
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

Find the configuration related to local_asr_model .

```python

# large_model_interface.yaml
## 离线语音识别 (Offline ASR)
local_asr_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/asr/SenseVoiceSmall"
# Local ASR model path
```

```python

```

```python
# large_model_interface.yaml
```

```python
## 离线语音识别 (Offline ASR)
```

```python
local_asr_model
:
"/home/jetson/yahboom_ws/src/largemodel/MODELS/asr/SenseVoiceSmall"
# Local ASR model path
```

### 3.2 Start and test the functionality

Startup Command :

```python

ros2 launch largemodel asr_only.launch.py
```

```python

```

```python
ros2 launch largemodel asr_only.launch.py
```

![image-20250807154240917](/img/docs/jetson/11-OfflineAIModel/11-18/1.png)
