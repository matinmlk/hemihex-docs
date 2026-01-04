---
title: Multimodal Visual Understanding Application
sidebar_position: 0
---

# Multimodal visual understand application

## 1. Concept Introduction

### 1.1 What is "Visual Understanding"?

In the largemodel project, the multimodal visual understanding feature enables robots to go beyond simply "seeing" a matrix of pixels and truly "understand" the content, objects, scenes, and relationships within an image. This is like giving robots a pair of thinking eyes.

The core tool for this feature is **seewhat`. When a user issues a command like "see what's here," the system invokes this tool, triggering a series of background operations that ultimately provide the user with AI-generated analysis of the live image in natural language.

### 1.2 Implementation Principle Overview

The basic principle is to input two different types of information— image (visual information) and text (linguistic information) —into a powerful multimodal large model (such as LLaVA).

Simply put, this involves highlighting the corresponding parts of the image with text, and then describing the highlighted parts with language .

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The seewhat function in this file defines the tool's execution flow.

[TODO1]

#### 2. Model Interface Layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file is the unified entry point for all image understanding tasks. It is responsible for calling the specific model implementation based on the configuration.

[TODO2]

### Code Analysis

This feature's implementation involves two primary layers: the tool layer defines the business logic, and the model interface layer is responsible for communicating with the large language model. This layered design is key to achieving platform versatility.

Tool layer ( tools_manager.py ) :

Model interface layer ( large_model_interface.py ) :

In summary, the execution flow of the seewhat tool embodies a clear separation of responsibilities: ToolsManager is responsible for defining "what to do" (obtaining images and requesting analysis), while model_interface is responsible for defining "how to do it" (selecting the appropriate model platform based on the current configuration and interacting with it). This makes the tutorial's interpretation universal; the core code logic remains consistent regardless of whether the user is in online or offline mode.

## 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in Terminal :

```bash

vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```


```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/confirm llm_platform :

```bash

model_service
:
#模型服务器节点参数 Model server node parameters
ros__parameters
:
language
:
'en'
#大模型接口语言 Large Model Interface Language
useolinetts
:
False
#文字模式下此项无效，可忽略 This item is invalid in text mode and can be ignored
​
# 大模型配置 Large model configuration
llm_platform
:
'ollama'
# 关键: 确保这里是 'ollama' Key: Make sure it's 'ollama'
regional_setting
:
"international"
```


```bash
model_service
:
#模型服务器节点参数 Model server node parameters
```

```bash
ros__parameters
:
```

```bash
language
:
'en'
#大模型接口语言 Large Model Interface Language
```

```bash
useolinetts
:
False
#文字模式下此项无效，可忽略 This item is invalid in text mode and can be ignored
```

```bash
​
```

```bash
# 大模型配置 Large model configuration
```

```bash
llm_platform
:
'ollama'
# 关键: 确保这里是 'ollama' Key: Make sure it's 'ollama'
```

```bash
regional_setting
:
"international"
```

#### 3.1.2 Configuring the Model Interface ( large_model_interface.yaml )

This file defines which vision model to use when the ollama platform is selected.

1.Open the file in Terminal

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
# Ollama配置 Ollama Configuration
ollama_host:
"http://localhost:11434"
# Ollama服务器地址 Ollama server address
ollama_model:
"llava"
# 关键: 将这里改为你已下载的多模态模型，如 "llava" Key: Change this to the multimodal model you downloaded, such as "llava"
#.....
```


```bash
#.....
```

```bash
## 离线大模型 (Offline Large Language Models)
```

```bash
# Ollama配置 Ollama Configuration
```

```bash
ollama_host:
"http://localhost:11434"
# Ollama服务器地址 Ollama server address
```

```bash
ollama_model:
"llava"
# 关键: 将这里改为你已下载的多模态模型，如 "llava" Key: Change this to the multimodal model you downloaded, such as "llava"
```

```bash
#.....
```

Note : Please make sure that the model specified in the configuration parameters (such as llava ) can handle multimodal input.

### 3.2 Starting and Testing the Function (Text Input Mode)

:::warning
Due to performance limitations, the performance of the Jetson Orin Nano 4GB is poor. To experience this function, please refer to the corresponding section in [Online Large Model (Text Interaction)]
:::

Start the largemodel main program (text mode) : Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py text_chat_mode:
=
true
```


```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:
=
true
```

Send text command : Open another terminal again and run the following command,

```bash

ros2 run text_chat text_chat
```


```bash
ros2 run text_chat text_chat
```

Then start typing the text: "What do you see".

Observation : In the first terminal where you run the main program, you will see log output indicating that the system received the text command, invoked the seewhat tool, and ultimately printed out the text description of the desktop generated by the LLaVA model.

## 4. Common Problems and Solutions

#### Problem 1: The log displays "Failed to call ollama vision model" or the connection is refused.

Solution :

#### Problem 2: The seewhat tool returns "Unable to open camera" or fails to capture.

Solution :
