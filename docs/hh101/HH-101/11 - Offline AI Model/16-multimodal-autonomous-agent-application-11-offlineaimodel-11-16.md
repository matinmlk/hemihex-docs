---
title: Multimodal Autonomous Agent Application
sidebar_position: 0
---

# Multimodal autonomous agent application

## 1. Concept Introduction

### 1.1 What is an "Autonomous Agent"?

In the largemodel project, multimodal autonomous agents represent the most advanced form of intelligence. Rather than simply responding to a user's command, they are capable of autonomously thinking, planning, and continuously invoking multiple tools to achieve a complex goal .

The core of this functionality is the **agent_call ** tool or its underlying **ToolChainManager . When a user issues a complex request that cannot be accomplished with a single tool call, the autonomous agent is activated.

### 1.2 Implementation Principles

The autonomous agent implementation in largemodel follows the industry-leading ReAct (Reason + Act) paradigm. Its core concept is to mimic the human problem-solving process, cycling between "thinking" and "acting."

This think -> act -> observe cycle continues until the initial goal is achieved, at which point the agent generates and outputs the final answer.

## 2. Code Analysis

### Key Code

#### 1. Agent Core Workflow ( largemodel/utils/ai_agent.py )

The _execute_agent_workflow function is the agent's main execution loop, defining the core "plan -> execute" process.
[TODO14]

#### 2. Mission planning and LLM interaction ( largemodel/utils/ai_agent.py )

The core of the _plan_task function is to build a sophisticated prompt and use the reasoning ability of the large model to generate a structured execution plan.

[TODO]

#### 3. Parameter processing and data flow implementation ( largemodel/utils/ai_agent.py )

The _process_step_parameters function is responsible for parsing placeholders and implementing data flow between steps.
[TODO]

### Code Analysis

The AI ​​Agent is the "brain" of the system, translating high-level, sometimes ambiguous, tasks posed by the user into a precise, ordered series of tool calls. Its implementation is independent of any specific model platform and built on a general, extensible architecture.

In summary, the general implementation of the AI ​​Agent demonstrates an advanced software architecture: rather than solving a problem directly, it builds a framework that enables an external, general-purpose reasoning engine (a large model) to solve the problem. Through two core mechanisms, dynamic programming and data flow management, the Agent orchestrates a series of independent tools into complex workflows capable of completing advanced tasks.

## 3. Practical Operations

### 3.1 Configuring the Offline Large Model

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

#### 3.1.2 Configuration model interface ( large_model_interface.yaml )

This file defines which visual model to use when the platform is selected as ollama .

```bash

vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```


```bash
vim ~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

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

### 3.2 Starting and Testing the Functionality (Text Input Mode)

:::warning
Due to performance limitations, the performance of the Jetson Orin Nano 4GB is limited. To experience this feature, please refer to the "Online Large Model (Text Interaction)" section.
:::

Start the largemodel main program (text mode): Open a terminal and run the following command:

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

Send text command : Open another terminal and run the following command:

```bash

ros2 run text_chat text_chat
```


```bash
ros2 run text_chat text_chat
```

Then start entering text: "Based on the current environment, save the generated environment description as a txt document."

Observe the Results :

In the first terminal running the main program, you will see log output indicating that the system receives the text command, invokes the aiagent tool, and then provides a prompt to LLM. LLM will analyze the detailed steps of the tool invocation. For example, in this question, the seewhat tool will be invoked to capture the image, which will then be parsed by LLM. The parsed text will be saved in the ~/yahboom_ws/src/largemodel/resources_file/documents folder.

## 4. Common Problems and Solutions

### 4.1 Abnormal Agent Behavior

#### Issue 1: The agent is stuck in an infinite loop or repeatedly executing the same tool.

Solution :

### 4.2 Tool Invocation Failure

#### Issue 2: The agent correctly planned the action, but the tool execution failed.

Solution :
