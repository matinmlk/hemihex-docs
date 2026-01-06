---
title: Qwen2.5VL
sidebar_position: 0
---

# Qwen2.5VL

Demo Environment

Development Board : Jetson Orin Series Motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB cannot run this model.
:::

Qwen2.5-VL is Qwen's new flagship visual language model and a significant leap forward compared to the previous Qwen2-VL.

## 1. Model Size

| Model | Volume |
| --- | --- |
| qwen2.5vl:3b | 3.2GB |
| qwen2.5vl:7b | 6.0GB |

## 2. Performance

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-7/c90b8e4b-d023-4953-9cd5-e515324ca73c.png)

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-7/533ec184-7b82-4bcf-a4ab-af9a41ac0810.png)

## 3. Using Qwen2.5VL

### 3.1 Running Qwen2.5VL

Use the run command to start running the model. If the model is not already downloaded, it will automatically pull the model from the Ollama model library:

```bash
ollama run qwen2.5vl:3b
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-7/1.png)

### 3.2 Start a conversation

```text
Please tell me how many hours there are in a day.
```

Response time depends on your hardware configuration. Please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-7/2.png)

### 3.3 Visual Function

![test_pic](/img/docs/jetson/11-OfflineAIModel/11-7/test_pic.png)

```text
What do you see in this picture? :./test_pic.png
#Use ": + image path" in the conversation to enable the model to use its visual function and interpret the information in the image.
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-7/3.png)

### 3.4 Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

Qwen2.5VL

GitHub: https://github.com/QwenLM/Qwen2.5-VL

Ollama Model: https://ollama.com/library/qwen2.5vl
