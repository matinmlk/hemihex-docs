---
title: Multimodal Visual Understanding Speech Interaction
sidebar_position: 0
---

# 4.Multimodal visual understand speech interaction

## 1. Concept Introduction

### 1.1 What is "Visual Understanding"?

In the largemodel project, the multimodal visual understanding feature enables robots to go beyond simply "seeing" a matrix of pixels and truly "understand" the content, objects, scenes, and relationships within an image. This is like giving robots a pair of thinking eyes.

The core tool for this feature is **seewhat`. When a user issues a command like "see what's here," the system invokes this tool, triggering a series of background operations that ultimately provide the user with AI-generated analysis of the live image in natural language.

### 1.2 Implementation Principle Overview

The basic principle is to input two different types of information— image (visual information) and text (linguistic information) —into a powerful multimodal large model (such as LLaVA).

Simply put, this involves highlighting the corresponding parts of the image with text, and then describing the highlighted parts with language .

## 2. Code Explanation

### Key Code

#### 1. Tools Layer Entry ( largemodel/utils/tools_manager.py )

The seewhat function in this file defines the execution flow of the tool.

```bash

# From largemodel/utils/tools_manager.py
​
class
ToolsManager
:
# ...
​
def
seewhat
(
self
):
"""
Capture camera frame and analyze environment with AI model.
捕获摄像头画面并使用AI模型分析环境。
:return: Dictionary with scene description and image path, or None if failed.
"""
self
.
node
.
get_logger
().
info
(
"Executing seewhat() tool"
)
image_path
=
self
.
capture_frame
()
if
image_path
:
# Use isolated context for image analysis. / 使用隔离的上下文进行图像分析。
analysis_text
=
self
.
_get_actual_scene_description
(
image_path
)
​
# Return structured data for the tool chain. / 为工具链返回结构化数据。
return
{
"description"
:
analysis_text
,
"image_path"
:
image_path
}
else
:
# ... (Error handling)
return
None
​
def
_get_actual_scene_description
(
self
,
image_path
,
message_context
=
None
):
"""
Get AI-generated scene description for captured image.
获取捕获图像的AI生成场景描述。
:param image_path: Path to captured image file.
:return: Plain text description of scene.
"""
try
:
# ... (构建Prompt)
# Force use of a plain text system prompt with a clean, one-time context. / 强制使用纯文本系统提示和干净的一次性上下文。
simple_context
= [{
"role"
:
"system"
,
"content"
:
"You are an image description assistant. ..."
}]
​
result
=
self
.
node
.
model_client
.
infer_with_image
(
image_path
,
scene_prompt
,
message
=
simple_context
)
# ... (处理结果)
return
description
except
Exception
as
e
:
# ...
```


```bash
# From largemodel/utils/tools_manager.py
```

```bash
​
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
​
```

```bash
def
seewhat
(
self
):
```

```bash
"""
```

```bash
Capture camera frame and analyze environment with AI model.
```

```bash
捕获摄像头画面并使用AI模型分析环境。
```

```bash
:return: Dictionary with scene description and image path, or None if failed.
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
"Executing seewhat() tool"
)
```

```bash
image_path
=
self
.
capture_frame
()
```

```bash
if
image_path
:
```

```bash
# Use isolated context for image analysis. / 使用隔离的上下文进行图像分析。
```

```bash
analysis_text
=
self
.
_get_actual_scene_description
(
image_path
)
```

```bash
​
```

```bash
# Return structured data for the tool chain. / 为工具链返回结构化数据。
```

```bash
return
{
```

```bash
"description"
:
analysis_text
,
```

```bash
"image_path"
:
image_path
```

```bash
}
```

```bash
else
:
```

```bash
# ... (Error handling)
```

```bash
return
None
```

```bash
​
```

```bash
def
_get_actual_scene_description
(
self
,
image_path
,
message_context
=
None
):
```

```bash
"""
```

```bash
Get AI-generated scene description for captured image.
```

```bash
获取捕获图像的AI生成场景描述。
```

```bash
:param image_path: Path to captured image file.
```

```bash
:return: Plain text description of scene.
```

```bash
"""
```

```bash
try
:
```

```bash
# ... (构建Prompt)
```

```bash
# Force use of a plain text system prompt with a clean, one-time context. / 强制使用纯文本系统提示和干净的一次性上下文。
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
"You are an image description assistant. ..."
```

```bash
}]
```

```bash
​
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
infer_with_image
(
image_path
,
scene_prompt
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
description
```

```bash
except
Exception
as
e
:
```

```bash
# ...
```

#### 2. Model interface layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file is the unified entry point for all image understanding tasks. It is responsible for calling the specific model implementation according to the configuration.

```bash

# From largemodel/utils/large_model_interface.py
​
class
model_interface
:
# ...
def
infer_with_image
(
self
,
image_path
,
text
=
None
,
message
=
None
):
"""Unified image inference interface. / 统一的图像推理接口。"""
# ... (准备消息)
try
:
# 根据 self.llm_platform 的值，决定调用哪个具体实现
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
image_path
=
image_path
)
elif
self
.
llm_platform
==
'tongyi'
:
# ... 调用通义模型的逻辑
pass
# ... (其他平台的逻辑)
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
infer_with_image
(
self
,
image_path
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
"""Unified image inference interface. / 统一的图像推理接口。"""
```

```bash
# ... (准备消息)
```

```bash
try
:
```

```bash
# 根据 self.llm_platform 的值，决定调用哪个具体实现
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
image_path
=
image_path
)
```

```bash
elif
self
.
llm_platform
==
'tongyi'
:
```

```bash
# ... 调用通义模型的逻辑
```

```bash
pass
```

```bash
# ... (其他平台的逻辑)
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

### Code Analysis

This feature's implementation involves two main layers: the tool layer defines the business logic, and the model interface layer is responsible for communicating with the large language model. This layered design is key to achieving platform versatility.

In summary, the seewhat tool's execution flow demonstrates a clear separation of responsibilities: ToolsManager defines the "what" (acquiring an image and requesting analysis), while model_interface defines the "how" (selecting the appropriate model platform based on the current configuration and interacting with it). This makes the tutorial's analysis universal, ensuring the core code logic remains consistent regardless of whether the user is online or offline.

## 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform ( HemiHex.yaml )

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash

vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```


```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/Confirm llm_platform :

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
True
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
True
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

Note : Please make sure that the model specified in the configuration parameters (such as llava ) can handle multimodal input.

### 3.2 Starting and Testing the Function

Note: Due to performance limitations, this example cannot be run on the Jetson Orin Nano 4GB. To experience this function, please refer to the corresponding section in [Online Large Model (Voice Interaction)]

Start the largemodel main program : Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py
```


```bash
ros2 launch largemodel largemodel_control.launch.py
```

After successful initialization, say the wake-up word and begin asking questions like: "What do you see?" or "Describe your current environment."

Observe the results : In the first terminal where the main program is running, you will see log output showing that the system receives the text command, invokes the seewhat tool, and ultimately prints the text description generated by the LLaVA model. The speaker will also announce the generated results.

## 4. Common Problems and Solutions

### 4.1 Very Slow Responses

Problem : After asking a question, it takes a long time for the voice response to arrive. Solution : The inference cost of a multimodal model is much higher than that of a text-only model, so higher latency is normal.
