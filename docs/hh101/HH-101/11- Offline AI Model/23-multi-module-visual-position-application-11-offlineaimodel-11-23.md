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
# ... (Path fallback mechanism and parameter checking)
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
# ... (Building an independent message context)
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
# ... (Process and parse the returned coordinate text)
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
# ... (Error Handling)
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
# ... (Path fallback mechanism and parameter checking)
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
# ... (Building an independent message context)
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
# ... (Process and parse the returned coordinate text)
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
# ... (Error Handling)
```

#### 2. Model interface layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file is the unified entry point for all image-related tasks.

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
# ... (Prepare Message)
try
:
# Determine which specific implementation to call based on the value of self.llm_platform
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
# ... Logic for calling the Tongyi model
pass
# ... (Logic of other platforms)
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
# ... (Prepare Message)
```

```bash
try
:
```

```bash
# Determine which specific implementation to call based on the value of self.llm_platform
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
# ... Logic for calling the Tongyi model
```

```bash
pass
```

```bash
# ... (Logic of other platforms)
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


```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/Confirm llm_platform :

```bash
model_service
:
#Model server node parameters
ros__parameters
:
language
:
'en'
#Large Model Interface Language
useolinetts
:
True
#This item is invalid in text mode and can be ignored
​
# Large model configuration
llm_platform
:
'ollama'
# Key: Make sure it's 'ollama'
regional_setting
:
"international"
```


```bash
model_service
:
#Model server node parameters
```

```bash
ros__parameters
:
```

```bash
language
:
'en'
#Large Model Interface Language
```

```bash
useolinetts
:
True
#This item is invalid in text mode and can be ignored
```

```bash
​
```

```bash
# Large model configuration
```

```bash
llm_platform
:
'ollama'
# Key: Make sure it's 'ollama'
```

```bash
regional_setting
:
"international"
```

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


```bash
ros2 launch largemodel largemodel_control.launch.py
```

Test :
