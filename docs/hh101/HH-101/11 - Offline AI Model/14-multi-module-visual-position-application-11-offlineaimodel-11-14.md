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
[TODO7]

#### 2. Model Interface Layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file serves as the unified entry point for all image-related tasks.
[TODO8]

### Code Analysis

The core of the visual positioning function lies in guiding large models to output structured data through precise instructions . It also follows a layered design with a tool layer and a model interface layer.

Tools layer ( tools_manager.py ) :

Model interface layer ( large_model_interface.py ) :

In summary, the general workflow for visual localization is: ToolsManager receives the target object name and constructs a precise prompt requesting coordinates. ToolsManager calls the model interface. ModelInterface packages the image and prompt together and sends them to the corresponding model platform according to the configuration. The model returns text containing the coordinates. ModelInterface returns this text to ToolsManager. ToolsManager parses the text, extracts the structured coordinate data, and returns it. This process demonstrates how Prompt Engineering can enable a general-purpose large visual model to accomplish more specific and structured tasks.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in terminal :

```bash
vim ~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```
[TODO9]

#### 3.1.2 Configuring the Model Interface ( large_model_interface.yaml )

This file defines which vision model to use when the ollama platform is selected.

1.Open the file in Terminal

```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```


2.Find the ollama related configuration
[TODO10]

Note : Please make sure that the model specified in the configuration parameters (such as llava ) can handle multimodal input.

### 3.2 Starting and Testing the Feature (Text Mode)

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB may not perform well. To experience this feature, please refer to the corresponding section in [Online Large Model (Text Interaction)]
:::

Prepare image files :

Place an image file to test in the following path: /home/jetson/yahboom_ws/src/largemodel/resources_file/visual_positioning

Then name the image test_image.jpg

Start the largemodel main program (in text mode):

Open a terminal and run the following command：

```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

Send text command : Open another terminal again and run the following command,

```bash
ros2 run text_chat text_chat
```
Then start typing text: "Analyze the position of the dinosaur in the picture."

Observations :In the first terminal where you run the main program, you'll see log output indicating that the system received the command, called the visual_positioning tool, completed the execution, and saved the coordinates to a file.

This file can be found in the ~/yahboom_ws/src/largemodel/resources_file/visual_positioning directory.

## 4. Common Problems and Solutions

#### Problem 1: "Image file not found" error message

Solution :
