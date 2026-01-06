---
title: Multi-module Visual Position Application
sidebar_position: 0
---

# Multi module visual position application

## 1. Introduction

### 1.1 What is "Multimodal Visual Localization"?

Multimodal visual localization is a technology that combines multiple sensor inputs (such as cameras, depth sensors, and IMUs) with algorithmic processing techniques to accurately identify and track the position and posture of a device or user in an environment. This technology does not rely solely on a single type of sensor data, but instead integrates information from different perception modalities, thereby improving localization accuracy and robustness.

### 1.2 Overview of Implementation Principles

## 2. Code Analysis

### Key Code

#### 1. Tools Layer Entry ( largemodel/utils/tools_manager.py )

The visual_positioning function in this file defines the execution flow of the tool, specifically how it constructs a prompt containing the target object name and formatting requirements.
[TODO]

### Code Analysis

The core of the visual positioning function lies in guiding large models to output structured data through precise instructions . It also follows the layered design of the tool layer and the model interface layer.

In summary, the general workflow for visual localization is: ToolsManager receives the target object name and constructs a precise prompt requesting coordinates. ToolsManager calls the model interface. ModelInterface packages the image and prompt together and sends them to the corresponding model platform according to the configuration. The model returns text containing the coordinates. ModelInterface returns this text to ToolsManager. ToolsManager parses the text, extracts the structured coordinate data, and returns it. This process demonstrates how prompt engineering techniques can be used to enable a general-purpose large-scale visual model to accomplish more specific and structured tasks.

## 3. Practical Application

### 3.1 Configuring the Offline Large-Scale Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large-scale model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

[TODO]

#### 3.1.2 Configuration model interface ( large_model_interface.yaml )

This file defines which visual model to use when the platform is selected as ollama .

```bash

vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```


```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

2.Find the ollama related configuration

```bash
#.....
## 离线大模型 (Offline Large Language Models)
# Ollama Configuration
ollama_host:
"http://localhost:11434"
# Ollama server address
ollama_model:
"llava"
# Key: Change this to the multimodal model you downloaded, such as "llava"
#.....
```

Note : Please ensure that the model specified in the configuration parameters (e.g., llava ) can handle multimodal input.

### 3.2 Starting and Testing the Function

Note: Due to performance limitations, this example cannot be run on the Jetson Orin Nano 4GB. To experience this feature, please refer to the corresponding section in [Online Large Model (Voice Interaction)]

Prepare image files :

Place an image file to test in the following path: /home/jetson/yahboom_ws/src/largemodel/resources_file/visual_positioning

Then name the image test_image.jpg

Start largemodel main program :

Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py
```

Test :
