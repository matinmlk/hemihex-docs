---
title: Multimodal Autonomous Proxy Application
sidebar_position: 25
---

# Multimodal Autonomous Proxy Application

## 1. Concept Introduction

### 1.1 What is an "Autonomous Agent"?

An **autonomous agent** is a software entity capable of independently perceiving its environment, making decisions, and executing actions to achieve a goal. In the `largemodel` project, the autonomous proxy agent integrates **multimodal perception**, **reasoning**, and **action execution** into a single closed-loop system.

This allows the system to accept high-level user instructions and autonomously determine the necessary steps to complete a task.

### 1.2 Implementation Principles

The multimodal autonomous proxy is built on three core components:

1. **Perception**  
   Collects multimodal inputs such as text commands, images, and system context.

2. **Decision Making (Reasoning)**  
   Uses a large language model to analyze the current state and determine the next action.

3. **Action Execution**  
   Executes system-level or robot-level commands through predefined tools.

This perception–decision–action loop enables continuous autonomous behavior.

---

## 2. Code Analysis

### Key Code

#### 2.1 Agent Core Workflow (`largemodel/utils/ai_agent.py`)

```python
class AIAgent:

    def run(self, user_input):
        """
        Main agent execution loop.
        """
        while True:
            decision = self.reason(user_input)
            if decision["type"] == "tool":
                self.call_tool(decision)
            elif decision["type"] == "response":
                return decision["content"]
```

#### 2.2 Reasoning Module

```python
def reason(self, input_text):
    """
    Analyze input and decide next action.
    """
    response = self.model.infer(input_text)
    return self.parse_response(response)
```

#### 2.3 Tool Invocation

```python
def call_tool(self, decision):
    """
    Execute a tool selected by the model.
    """
    tool_name = decision["tool"]
    return self.tools_manager.execute(tool_name, decision["args"])
```

### Architecture Summary

- **Agent Layer**: Controls task planning and execution.
- **Model Interface Layer**: Handles communication with the large language model.
- **Tool Layer**: Provides callable system and robot functions.

This layered design enables extensibility and platform independence.

---

## 3. Offline Model Configuration

### 3.1 LLM Platform Configuration (`hemihex.yaml`)

```yaml
model_service:
  ros__parameters:
    llm_platform: "ollama"
    language: "en"
```

### 3.2 Ollama Configuration

```yaml
# Offline Large Language Models
ollama_host: "http://localhost:11434"
ollama_model: "llava"
```

:::note
Ensure the configured model supports multimodal reasoning and tool calling.
:::

---

## 4. Running the Autonomous Agent

1. **Start the largemodel service**:

```bash
ros2 launch largemodel largemodel_control.launch.py
```

2. **Send a high-level command**:

```bash
ros2 run text_chat text_chat
```

Example input:

```text
Analyze the environment and decide what to do next
```

3. **Expected behavior**  
The agent reasons about the request, selects tools as needed, and autonomously executes actions.

---

## 5. Common Problems and Solutions

### Problem 1: Agent loops indefinitely

**Solution**
- Verify tool responses are correctly formatted.
- Ensure stop conditions are defined in prompts.

### Problem 2: Tool execution fails

**Solution**
- Check tool registration.
- Verify permissions and system dependencies.

---

This documentation is maintained by **HemiHex** and describes the offline multimodal autonomous proxy application architecture.
