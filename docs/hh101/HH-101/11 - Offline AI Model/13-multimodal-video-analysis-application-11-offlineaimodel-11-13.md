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

```bash
xxxxxxxxxx
# From largemodel/utils/tools_manager.py
class
ToolsManager
:
# ...
def
analyze_video
(
self
,
args
):
"""
Analyze video file and provide content description.
分析视频文件并提供内容描述。
:param args: Arguments containing video path.
:return: Dictionary with video description and path.
"""
self
.
node
.
get_logger
().
info
(
f"Executing analyze_video() tool with args: {args}"
)
try
:
video_path
=
args
.
get
(
"video_path"
)
# ... (智能路径回退机制)
if
video_path
and
os
.
path
.
exists
(
video_path
):
# ... (构建Prompt)
# Use a fully isolated, one-time context for video analysis to ensure a plain text description. / 使用完全隔离的一次性上下文进行视频分析，以确保获得纯文本描述。
simple_context
= [{
"role"
:
"system"
,
"content"
:
"You are a video description assistant. ..."
}]
result
=
self
.
node
.
model_client
.
infer_with_video
(
video_path
,
prompt
,
message
=
simple_context
)
# ... (处理结果)
return
{
"description"
:
description
,
"video_path"
:
video_path
}
# ... (错误处理)
```

```bash
xxxxxxxxxx
```

```bash
# From largemodel/utils/tools_manager.py
```

```bash
class
ToolsManager
:
```

```bash
# ...
```

```bash
def
analyze_video
(
self
,
args
):
```

```bash
"""
```

```bash
Analyze video file and provide content description.
```

```bash
分析视频文件并提供内容描述。
```

```bash
:param args: Arguments containing video path.
```

```bash
:return: Dictionary with video description and path.
```

```bash
"""
```

```bash
self
.
node
.
get_logger
().
info
(
f"Executing analyze_video() tool with args: {args}"
)
```

```bash
try
:
```

```bash
video_path
=
args
.
get
(
"video_path"
)
```

```bash
# ... (智能路径回退机制)
```

```bash
if
video_path
and
os
.
path
.
exists
(
video_path
):
```

```bash
# ... (构建Prompt)
```

```bash
# Use a fully isolated, one-time context for video analysis to ensure a plain text description. / 使用完全隔离的一次性上下文进行视频分析，以确保获得纯文本描述。
```

```bash
simple_context
= [{
```

```bash
"role"
:
"system"
,
```

```bash
"content"
:
"You are a video description assistant. ..."
```

```bash
}]
```

```bash
result
=
self
.
node
.
model_client
.
infer_with_video
(
video_path
,
prompt
,
message
=
simple_context
)
```

```bash
# ... (处理结果)
```

```bash
return
{
```

```bash
"description"
:
description
,
```

```bash
"video_path"
:
video_path
```

```bash
}
```

```bash
# ... (错误处理)
```

#### 2. Model Interface Layer and Frame Extraction ( largemodel/utils/large_model_interface.py )

The functions in this file are responsible for processing video files and passing them to the underlying model.

```bash
xxxxxxxxxx
# From largemodel/utils/large_model_interface.py
​
class
model_interface
:
# ...
def
infer_with_video
(
self
,
video_path
,
text
=
None
,
message
=
None
):
"""Unified video inference interface. / 统一的视频推理接口。"""
# ... (准备消息)
try
:
# 根据 self.llm_platform 决定调用哪个具体实现
if
self
.
llm_platform
==
'ollama'
:
response_content
=
self
.
ollama_infer
(
self
.
messages
,
video_path
=
video_path
)
# ... (其他在线平台的逻辑)
# ...
return
{
'response'
:
response_content
,
'messages'
:
self
.
messages
.
copy
()}
​
def
_extract_video_frames
(
self
,
video_path
,
max_frames
=
5
):
"""Extract keyframes from a video for analysis. / 从视频中提取关键帧用于分析。"""
try
:
import
cv2
# ... (视频读取和帧间隔计算)
while
extracted_count
<
max_frames
:
# ... (循环读取视频帧)
if
frame_count
%
frame_interval
==
0
:
# ... (将帧保存为临时图片)
frame_base64
=
self
.
encode_file_to_base64
(
temp_path
)
frame_images
.
append
(
frame_base64
)
# ...
return
frame_images
# ... (异常处理)
```

```bash
xxxxxxxxxx
```

```bash
# From largemodel/utils/large_model_interface.py
```

```bash
​
```

```bash
class
model_interface
:
```

```bash
# ...
```

```bash
def
infer_with_video
(
self
,
video_path
,
text
=
None
,
message
=
None
):
```

```bash
"""Unified video inference interface. / 统一的视频推理接口。"""
```

```bash
# ... (准备消息)
```

```bash
try
:
```

```bash
# 根据 self.llm_platform 决定调用哪个具体实现
```

```bash
if
self
.
llm_platform
==
'ollama'
:
```

```bash
response_content
=
self
.
ollama_infer
(
self
.
messages
,
video_path
=
video_path
)
```

```bash
# ... (其他在线平台的逻辑)
```

```bash
# ...
```

```bash
return
{
'response'
:
response_content
,
'messages'
:
self
.
messages
.
copy
()}
```

```bash
​
```

```bash
def
_extract_video_frames
(
self
,
video_path
,
max_frames
=
5
):
```

```bash
"""Extract keyframes from a video for analysis. / 从视频中提取关键帧用于分析。"""
```

```bash
try
:
```

```bash
import
cv2
```

```bash
# ... (视频读取和帧间隔计算)
```

```bash
while
extracted_count
<
max_frames
:
```

```bash
# ... (循环读取视频帧)
```

```bash
if
frame_count
%
frame_interval
==
0
:
```

```bash
# ... (将帧保存为临时图片)
```

```bash
frame_base64
=
self
.
encode_file_to_base64
(
temp_path
)
```

```bash
frame_images
.
append
(
frame_base64
)
```

```bash
# ...
```

```bash
return
frame_images
```

```bash
# ... (异常处理)
```

### Code Analysis

The implementation of video analysis is more complex than image analysis. It requires a key preprocessing step at the model interface layer: frame extraction.

In summary, the general process of video analysis is: ToolsManager initiates an analysis request -> model_interface intercepts the request and calls _extract_video_frames to decompose the video file into multiple keyframe images -> model_interface sends these images, along with analysis instructions, to the corresponding model platform according to the configuration -> the model returns a comprehensive description of the video -> the results are finally returned to ToolsManager . This design ensures the stability and versatility of upper-layer applications.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash
xxxxxxxxxx
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

```bash
xxxxxxxxxx
```

```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/confirm llm_platform :

```bash
xxxxxxxxxx
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
xxxxxxxxxx
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

This file defines which visual model to use when the ollama platform is selected.

```bash
xxxxxxxxxx
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
xxxxxxxxxx
```

```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
xxxxxxxxxx
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

```bash
xxxxxxxxxx
```

```bash
#.....
```

```bash
## 离线大模型 (Offline Large Language Models)
```

```bash
# Ollama Configuration
```

```bash
ollama_host:
"http://localhost:11434"
# Ollama server address
```

```bash
ollama_model:
"llava"
# Key: Change this to the multimodal model you downloaded, such as "llava"
```

```bash
#.....
```

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
xxxxxxxxxx
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

```bash
xxxxxxxxxx
```

```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

Send text command : Open another terminal again and run the following command,

```bash
xxxxxxxxxx
ros2 run text_chat text_chat
```

```bash
xxxxxxxxxx
```

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
