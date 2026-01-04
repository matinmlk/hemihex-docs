---
title: Llava
sidebar_position: 0
---

# Llava

:::note
Demonstration Environment
:::

Demonstration Environment

Development Board : Jetson Orin Series Motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB cannot run this model.
:::

## 1. Model Scale

| Model | Volume |
| --- | --- |
| llava:7b | 4.7GB |

## 2. Performance

![img](/img/docs/jetson/11-OfflineAIModel/11-9/pie_llava_gpt4.png)

## 3. Using Llava

### 3.1 Running Llava

Use the run command to start running the model. If the model has not been downloaded, it will automatically pull the model from the Ollama model library:

```text
xxxxxxxxxx
ollama run llava:7b
```

```text
xxxxxxxxxx
```

```bash
ollama run llava:7b
```

![image-20250630141438495](/img/docs/jetson/11-OfflineAIModel/11-9/1.png)

### 3.2 Starting a Conversation

```text
xxxxxxxxxx
How many minutes is half an hour?
```

```text
xxxxxxxxxx
```

```text
How many minutes is half an hour?
```

Response time depends on your hardware configuration. Please be patient!

![image-20250630143938794](/img/docs/jetson/11-OfflineAIModel/11-9/image-20250630143938794.png)

### 3.3 Visual Function

![test_pic](/img/docs/jetson/11-OfflineAIModel/11-9/test_pic.png)

```text
xxxxxxxxxx
What do you see in this picture? :./test_pic.png
#Use ": + image path" in the conversation to enable the model to use its visual function and interpret the information in the image.
```

```text
xxxxxxxxxx
```

```text
What do you see in this picture? :./test_pic.png
```

```text
#Use ": + image path" in the conversation to enable the model to use its visual function and interpret the information in the image.
```

![image-20250702095903866](/img/docs/jetson/11-OfflineAIModel/11-9/2.png)

### 3.4 Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

Llava

Ollama Model: https://ollama.com/library/llava
