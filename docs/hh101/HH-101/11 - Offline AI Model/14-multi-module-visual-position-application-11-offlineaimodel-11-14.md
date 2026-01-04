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

```bash
# From largemodel/utils/tools_manager.py
class
ToolsManager
:
# ...
def
visual_positioning
(
self
,
args
):
"""
Locate object coordinates in image and save results to MD file.
定位图像中物体坐标并将结果保存为MD文件。
:param args: Arguments containing image path and object name.
:return: Dictionary with file path and coordinate data.
"""
self
.
node
.
get_logger
().
info
(
f"Executing visual_positioning() tool with args: {args}"
)
try
:
image_path
=
args
.
get
(
"image_path"
)
object_name
=
args
.
get
(
"object_name"
)
# ... (路径回退机制和参数检查)
# Construct a prompt asking the large model to identify the coordinates of the specified object. / 构造提示，要求大模型识别指定物品的坐标。
if
self
.
node
.
language
==
'zh'
:
prompt
=
f"请仔细分析这张图片，用一个个框定位图像每一个{object_name}的位置..."
else
:
prompt
=
f"Please carefully analyze this image and find the position of all {object_name}..."
# ... (构建独立的message上下文)
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
prompt
,
message
=
message_to_use
)
# ... (处理和解析返回的坐标文本)
return
{
"file_path"
:
md_file_path
,
"coordinates_content"
:
coordinates_content
,
"explanation_content"
:
explanation_content
}
# ... (错误处理)
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
visual_positioning
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
Locate object coordinates in image and save results to MD file.
```

```bash
定位图像中物体坐标并将结果保存为MD文件。
```

```bash
:param args: Arguments containing image path and object name.
```

```bash
:return: Dictionary with file path and coordinate data.
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
f"Executing visual_positioning() tool with args: {args}"
)
```

```bash
try
:
```

```bash
image_path
=
args
.
get
(
"image_path"
)
```

```bash
object_name
=
args
.
get
(
"object_name"
)
```

```bash
# ... (路径回退机制和参数检查)
```

```bash
# Construct a prompt asking the large model to identify the coordinates of the specified object. / 构造提示，要求大模型识别指定物品的坐标。
```

```bash
if
self
.
node
.
language
==
'zh'
:
```

```bash
prompt
=
f"请仔细分析这张图片，用一个个框定位图像每一个{object_name}的位置..."
```

```bash
else
:
```

```bash
prompt
=
f"Please carefully analyze this image and find the position of all {object_name}..."
```

```bash
# ... (构建独立的message上下文)
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
prompt
,
message
=
message_to_use
)
```

```bash
# ... (处理和解析返回的坐标文本)
```

```bash
return
{
```

```bash
"file_path"
:
md_file_path
,
```

```bash
"coordinates_content"
:
coordinates_content
,
```

```bash
"explanation_content"
:
explanation_content
```

```bash
}
```

```bash
# ... (错误处理)
```

#### 2. Model Interface Layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file serves as the unified entry point for all image-related tasks.

```bash
​
x
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
x
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
x
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
xxxxxxxxxx
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
xxxxxxxxxx
```

```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

2.Find the ollama related configuration

```bash
xxxxxxxxxx
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
xxxxxxxxxx
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

Then start typing text: "Analyze the position of the dinosaur in the picture."

Observations :In the first terminal where you run the main program, you'll see log output indicating that the system received the command, called the visual_positioning tool, completed the execution, and saved the coordinates to a file.

This file can be found in the ~/yahboom_ws/src/largemodel/resources_file/visual_positioning directory.

## 4. Common Problems and Solutions

#### Problem 1: "Image file not found" error message

Solution :
