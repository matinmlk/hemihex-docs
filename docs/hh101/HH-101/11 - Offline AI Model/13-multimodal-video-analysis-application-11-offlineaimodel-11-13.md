---
title: Multimodal Video Analysis Application
sidebar_position: 0
---

# Multimodal Video Analysis Application

## 1. Concept Introduction

### 1.1 What is "Video Analysis"?

In the largemodel project, the multimodal video analysis feature enables a robot to process a video and summarize its core content, describe key events, or answer specific questions about the video in natural language. This allows the robot to leap from understanding only static images to understanding the dynamic and temporal world.

The core tool for this feature is **analyze_video`. When a user provides a video file and asks a question (such as "Summarize what this video says"), the system invokes this tool to process and analyze the video and return a textual response from the AI.

### 1.2 Implementation Principles

The core challenge of offline video analysis lies in how to efficiently process video data containing hundreds or thousands of frames. A popular implementation principle is as follows:

Simply put, it condenses a video into a few key images and their sequence, allowing users to understand the entire story like reading a comic strip and answer related questions.

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The analyze_video function in this file defines the tool's execution flow.

[TODO3]

#### 2. Model Interface Layer and Frame Extraction ( largemodel/utils/large_model_interface.py )

The functions in this file are responsible for processing video files and passing them to the underlying model.

[TODO4]

### Code Analysis

The implementation of video analysis is more complex than image analysis. It requires a key preprocessing step at the model interface layer: frame extraction.

In summary, the general process of video analysis is: ToolsManager initiates an analysis request -> model_interface intercepts the request and calls _extract_video_frames to decompose the video file into multiple keyframe images -> model_interface sends these images, along with analysis instructions, to the corresponding model platform according to the configuration -> the model returns a comprehensive description of the video -> the results are finally returned to ToolsManager . This design ensures the stability and versatility of upper-layer applications.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash
vim ~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/confirm llm_platform :

[TODO5]

#### 3.1.2 Configuring the Model Interface ( large_model_interface.yaml )

This file defines which visual model to use when the ollama platform is selected.

```bash

vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```
[TODO6]

Note : Please ensure that the model specified in the configuration parameters (e.g., llava ) can handle multimodal input.

### 3.2 Starting and Testing the Feature (Text Input Mode)

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB may not perform well. To experience this feature, please refer to the corresponding section in [Online Large Model (Text Interaction)]
:::

Prepare video files :

Place a video file to test in the following path: /home/jetson/yahboom_ws/src/largemodel/resources_file/analyze_video

Then name the video test_video.mp4

Start the largemodel main program (in text mode):

Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```


Send text command : Open another terminal again and run the following command,

```bash
ros2 run text_chat text_chat
```


Then start typing text: "Analyze this video."

Observation results : In the first terminal running the main program, you will see log output showing that the system received the command, called the analyze_video tool, extracted the keyframes, and finally printed out the AI's summary of the video content.

## 4. Common Problems and Solutions

#### Problem 1: Error message "Video file not found" or "Unable to extract keyframes from video."

Solution :

#### Problem 2: Analyzing a long video is time-consuming.

Solution :
