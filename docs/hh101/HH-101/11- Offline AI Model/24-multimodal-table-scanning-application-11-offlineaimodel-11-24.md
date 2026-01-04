---
title: Multimodal Table Scanning Application
sidebar_position: 0
---

# Multimodal table scanning application

## 1. Concept Introduction

### 1.1 What is "Multimodal Table Scanning"?

Multimodal table scanning is a technology that uses image processing and artificial intelligence to identify and extract table information from images or PDF documents. It not only focuses on visual table structure recognition but also incorporates multimodal data such as text content and layout information to enhance table understanding. Large Language Models (LLMs) provide powerful semantic analysis capabilities to understand this extracted information. The two complement each other and enhance the intelligence of document processing.

### 1.2 Implementation Principle Overview

Table Detection and Content Recognition

Multimodal Fusion

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The scan_table function in this file defines the tool's execution flow, specifically how it constructs a prompt that returns a Markdown-formatted result.

```bash

# From largemodel/utils/tools_manager.py
class
ToolsManager
:
# ...
def
scan_table
(
self
,
args
):
"""
Scan a table from an image and save the content as a Markdown file.
从图像中扫描表格，并将内容保存为Markdown文件。
​
:param args: Arguments containing the image path.
:return: Dictionary with file path and content.
"""
self
.
node
.
get_logger
().
info
(
f"Executing scan_table() tool with args: {args}"
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
# ... (Path checking and fallback)
​
# Construct a prompt asking the large model to recognize the table and return it in Markdown format.
# 构造提示，要求大模型识别表格并以Markdown格式返回。
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
"请仔细分析这张图片，识别其中的表格，并将其内容以Markdown格式返回。"
else
:
prompt
=
"Please carefully analyze this image, identify the table within it, and return its content in Markdown format."
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
prompt
)
# ... (Extract Markdown text from the results)
​
# Save the recognized content to a Markdown file. / 将识别出的内容保存到Markdown文件。
md_file_path
=
os
.
path
.
join
(
self
.
node
.
pkg_path
,
"resources_file"
,
"scanned_tables"
,
f"table_{timestamp}.md"
)
with
open
(
md_file_path
,
'w'
,
encoding
=
'utf-8'
)
as
f
:
f
.
write
(
table_content
)
​
return
{
"file_path"
:
md_file_path
,
"table_content"
:
table_content
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
scan_table
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
Scan a table from an image and save the content as a Markdown file.
```

```bash
从图像中扫描表格，并将内容保存为Markdown文件。
```

```bash
​
```

```bash
:param args: Arguments containing the image path.
```

```bash
:return: Dictionary with file path and content.
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
f"Executing scan_table() tool with args: {args}"
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
# ... (Path checking and fallback)
```

```bash
​
```

```bash
# Construct a prompt asking the large model to recognize the table and return it in Markdown format.
```

```bash
# 构造提示，要求大模型识别表格并以Markdown格式返回。
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
"请仔细分析这张图片，识别其中的表格，并将其内容以Markdown格式返回。"
```

```bash
else
:
```

```bash
prompt
=
"Please carefully analyze this image, identify the table within it, and return its content in Markdown format."
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
prompt
)
```

```bash
# ... (Extract Markdown text from the results)
```

```bash
​
```

```bash
# Save the recognized content to a Markdown file. / 将识别出的内容保存到Markdown文件。
```

```bash
md_file_path
=
os
.
path
.
join
(
self
.
node
.
pkg_path
,
"resources_file"
,
"scanned_tables"
,
f"table_{timestamp}.md"
)
```

```bash
with
open
(
md_file_path
,
'w'
,
encoding
=
'utf-8'
)
as
f
:
```

```bash
f
.
write
(
table_content
)
```

```bash
​
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
"table_content"
:
table_content
```

```bash
}
```

```bash
# ... (Error Handling)
```

#### 2. Model Interface Layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file serves as the unified entry point for all image-related tasks.

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

The table scanning function is a typical application for converting unstructured image data into structured text data. Its core technology remains guiding model behavior through prompt engineering .

Tools Layer ( tools_manager.py ) :

Model Interface Layer ( large_model_interface.py ) :

In summary, the general workflow for table scanning is: ToolsManager receives an image and constructs a command to convert the table in this image to Markdown. ToolsManager calls the model interface. Model_interface packages the image and the command and sends them to the corresponding model platform according to the configuration. The model returns Markdown-formatted text. Model_interface returns the text to ToolsManager. ToolsManager saves the text as a .md file and returns the result. This workflow demonstrates how to leverage the formatting capabilities of a large model as a powerful OCR (Optical Character Recognition) and data structuring tool.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

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

#### 3.1.2 Configuring the Model Interface ( large_model_interface.yaml )

This file defines which visual model to use when the ollama platform is selected.

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

Note: Due to performance limitations, this example cannot be run on a Jetson Orin Nano 4GB. To experience this feature, please refer to the corresponding section in [Online Large Model (Voice Interaction)]

Prepare the Image File :

Place an image file to test in the following path: /home/jetson/yahboom_ws/src/largemodel/resources_file/scan_table

Then name the image test_table.jpg

Start the largemodel main program :

Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py
```


```bash
ros2 launch largemodel largemodel_control.launch.py
```

Test :
