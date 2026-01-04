---
title: Multimodal Visual Understanding Application
sidebar_position: 2
---

# 2.Multimodal visual understand application

## 1. Concept Introduction

### 1.1 What is "Video Analysis"?

In the largemodel project, the multimodal video analysis feature enables a robot to process a video and summarize its core content, describe key events, or answer specific questions about the video in natural language. This allows the robot to leap from understanding only static images to understanding dynamic and temporal relationships.

The core tool for this feature is analyze_video . When a user provides a video file and asks a question (such as "Summarize what this video says"), the system invokes this tool to process and analyze the video and return a textual response from the AI.

### 1.2 Implementation Principles

The core challenge of offline video analysis lies in how to efficiently process video data containing hundreds or thousands of frames. A popular implementation principle is as follows:

Simply put, it is to condense a video into several key pictures and their sequence, and then understand the whole story like reading a comic strip and answer related questions .

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The analyze_video function in this file defines the execution flow of the tool.

```yaml
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
Analyzes video files and provides a description of the content.
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
# ... (Intelligent path fallback mechanism)
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
# ... (Build Prompt)
# Use a fully isolated, one-time context for video analysis to ensure a plain text description.
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
# ... (processing results)
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
# ... (error handling)
```

```bash
# From largemodel/utils/tools_manager.py
```

```yaml
class
ToolsManager
:
```

```bash
# ...
```

```yaml
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
Analyzes video files and provides a description of the content.
```

```yaml
:param args: Arguments containing video path.
```

```yaml
:return: Dictionary with video description and path.
```

```bash
"""
```

```yaml
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

```yaml
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
# ... (Intelligent path fallback mechanism)
```

```yaml
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
# ... (Build Prompt)
```

```bash
# Use a fully isolated, one-time context for video analysis to ensure a plain text description.
```

```bash
simple_context
= [{
```

```yaml
"role"
:
"system"
,
```

```yaml
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
# ... (processing results)
```

```bash
return
{
```

```yaml
"description"
:
description
,
```

```yaml
"video_path"
:
video_path
```

```bash
}
```

```bash
# ... (error handling)
```

#### 2. Model interface layer and frame extraction ( largemodel/utils/large_model_interface.py )

The functions in this file are responsible for processing the video files and passing them to the underlying model.

```yaml
​
x
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
"""Unified video inference interface. """
# ... (prepare message)
try
:
# Determine which specific implementation to call based on self.llm_platform
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
# ... (Logic for other online platforms)
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
"""Extract keyframes from a video for analysis. """
try
:
import
cv2
# ... (video reading and frame interval calculation)
while
extracted_count
<
max_frames
:
# ... (Loop reading video frames)
if
frame_count
%
frame_interval
==
0
:
# ... (save the frame as a temporary image)
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
# ... (Exception handling)
```

```bash
# From largemodel/utils/large_model_interface.py
```

```bash
​
```

```yaml
class
model_interface
:
```

```bash
# ...
```

```yaml
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
"""Unified video inference interface. """
```

```bash
# ... (prepare message)
```

```yaml
try
:
```

```bash
# Determine which specific implementation to call based on self.llm_platform
```

```yaml
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
# ... (Logic for other online platforms)
```

```bash
# ...
```

```yaml
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

```yaml
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
"""Extract keyframes from a video for analysis. """
```

```yaml
try
:
```

```bash
import
cv2
```

```bash
# ... (video reading and frame interval calculation)
```

```yaml
while
extracted_count
<
max_frames
:
```

```bash
# ... (Loop reading video frames)
```

```yaml
if
frame_count
%
frame_interval
==
0
:
```

```bash
# ... (save the frame as a temporary image)
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
# ... (Exception handling)
```

### Code Analysis

The implementation of video analysis is more complex than image analysis. It requires a key preprocessing step at the model interface layer: frame extraction.

Tools Layer ( tools_manager.py ) :

Model Interface Layer ( large_model_interface.py ) :

In summary, the general process of video analysis is: ToolsManager initiates an analysis request -> model_interface intercepts the request and calls _extract_video_frames to decompose the video file into multiple keyframe images -> model_interface sends these images, along with analysis instructions, to the corresponding model platform according to the configuration -> the model returns a comprehensive description of the video -> the results are finally returned to ToolsManager . This design ensures the stability and versatility of upper-layer applications.

## 3. Practical Operations

### 3.1 Configuring Online LLM

First, obtain an API key from any platform described in the previous tutorial

Then, you need to update the key in the configuration file and open the model interface configuration file. large_model_interface.yaml :

```bash
xxxxxxxxxx
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
xxxxxxxxxx
```

```bash
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

Enter your API Key : Find the corresponding section and paste the API Key you just copied. This example uses the Tongyi Qianwen configuration.

```yaml
x
# large_model_interface.yaml
​
## Thousand Questions on Tongyi
qianwen_api_key
:
"sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
# Paste your Key
qianwen_model
:
"qwen-vl-max-latest"
# You can choose the model as needed, such as qwen-turbo, qwen-plus
```

```bash
x
```

```bash
# large_model_interface.yaml
```

```bash
​
```

```bash
## Thousand Questions on Tongyi
```

```yaml
qianwen_api_key
:
"sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
# Paste your Key
```

```yaml
qianwen_model
:
"qwen-vl-max-latest"
# You can choose the model as needed, such as qwen-turbo, qwen-plus
```

Open the main configuration file HemiHex.yaml :

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

Select the online platform you want to use : Change the llm_platform parameter to the platform name you want to use.

```yaml
xxxxxxxxxx
# HemiHex.yaml
​
model_service
:
ros__parameters
:
# ...
llm_platform
:
'tongyi'
#Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'
```

```bash
xxxxxxxxxx
```

```bash
# HemiHex.yaml
```

```bash
​
```

```yaml
model_service
:
```

```yaml
ros__parameters
:
```

```bash
# ...
```

```yaml
llm_platform
:
'tongyi'
#Optional platforms: 'ollama', 'tongyi', 'spark', 'qianfan', 'openrouter'
```

### 3.2 Launching and Testing the Functionality

Launch the largemodel main program and enable text interaction mode :

```yaml
xxxxxxxxxx
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:
=
true
```

```bash
xxxxxxxxxx
```

```yaml
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:
=
true
```

Send text command : Open another terminal and run the following command:

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

Then, start typing your question.

Test :
