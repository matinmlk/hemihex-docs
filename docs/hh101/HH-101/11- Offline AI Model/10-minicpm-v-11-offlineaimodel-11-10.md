---
title: MiniCPM-V
sidebar_position: 0
---

# MiniCPM-V

Demonstration Environment

Development Board: Jetson Orin Series

:::warning
Due to performance limitations, this model cannot be run on the Jetson Orin Nano 4GB.
:::

## 1. Model Size

| Model | Size |
| --- | --- |
| minicpm-v:8b | 5.5GB |

## 2. Performance

![img](/img/docs/jetson/11-OfflineAIModel/11-10/radar.jpg)

## 3. Using MiniCPM-V

### 3.1 Running MiniCPM-V

If the system does not have a running model, the system will automatically pull the TinyLlama 1.1B model and run it:

```bash
ollama run minicpm-v:8b
```

### 3.2 Starting a Conversation

```bash
Tell me a mathematician story
```

Response time depends on hardware configuration, so please be patient!

![image-20250630153524527](/img/docs/jetson/11-OfflineAIModel/11-10/image-20250630153524527.png)

### 3.3 Visual Function

![test_pic](/img/docs/jetson/11-OfflineAIModel/11-10/test_pic.png)

```bash

What do you see in this picture? :./test_pic.png
#Use ": + image path" in the conversation to enable the model to use its visual function and interpret the information in the image.
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-10/1.png)

### 3.4 Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

MiniCPM-V

GitHub: https://github.com/OpenBMB/MiniCPM-o

Ollama Model: https://ollama.com/library/minicpm-v
