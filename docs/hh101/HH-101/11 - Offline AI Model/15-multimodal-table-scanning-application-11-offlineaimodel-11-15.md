---
title: Multimodal Table Scanning Application
sidebar_position: 0
---

# Multimodal table scanning application

## 1. Concept Introduction

### 1.1 What is "Multimodal Table Scanning"?

Multimodal table scanning is a technology that uses image processing and artificial intelligence to identify and extract table information from images or PDF documents. It not only focuses on visual table structure recognition but also incorporates multimodal data such as text content and layout information to enhance table understanding. Large Language Models (LLMs) provide powerful semantic analysis capabilities to understand this extracted information. The two complement each other and enhance the intelligence of document processing.

### 1.2 Implementation Principle Overview

## 2. Code Analysis

### Key Code

#### 1. Tool Layer Entry ( largemodel/utils/tools_manager.py )

The scan_table function in this file defines the tool's execution flow, specifically how it constructs a prompt that returns a Markdown-formatted result.
[TODO11]

#### 2. Model Interface Layer ( largemodel/utils/large_model_interface.py )

The infer_with_image function in this file serves as the unified entry point for all image-related tasks.

[TODO12]

### Code Analysis

The table scanning function is a typical application for converting unstructured image data into structured text data. Its core technology remains guiding model behavior through prompt engineering .

In summary, the general workflow for table scanning is: ToolsManager receives an image and constructs a command to convert the table in this image to Markdown. ToolsManager calls the model interface. model_interface packages the image and the command and sends it to the corresponding model platform according to the configuration. The model returns Markdown-formatted text. model_interface returns the text to ToolsManager. ToolsManager saves the text as a .md file and returns the result. This workflow demonstrates how to leverage the formatting capabilities of a large model as a powerful OCR (Optical Character Recognition) and data structuring tool.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

#### 3.1.1 Configuring the LLM Platform (HemiHex.yaml)

This file determines which large model platform the model_service node loads as its primary language model.

Open the file in the terminal :

```bash
vim ~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

Modify/Confirm llm_platform :


Note : Please ensure that the model specified in the configuration parameters (e.g., llava ) can handle multimodal input.

### 3.2 Starting and Testing (Text Mode)

:::warning
Due to performance limitations on the Jetson Orin Nano 4GB, performance may be poor. To experience this feature, please refer to the corresponding section in [Online Large Model (Text Interaction)].
:::

Place a table image file to test in the following path: /home/jetson/yahboom_ws/src/largemodel/resources_file/scan_table

Then name the image test_table.jpg

Open a terminal and run the following command:

```bash

ros2 launch largemodel largemodel_control.launch.py
•text_chat_mode:=true
```

Then start typing: "Analyze the table."

This document can be found in the ~/yahboom_ws/src/largemodel/resources_file/scan_table directory.

## 4. Common Problems and Solutions

#### Problem 1: Incomplete recognition of table content or typos.

Solution :

#### Problem 2: "Table file not found" error

Solution :
