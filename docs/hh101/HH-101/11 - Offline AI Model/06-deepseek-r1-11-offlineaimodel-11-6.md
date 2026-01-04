---
title: DeepSeek-R1
sidebar_position: 0
---

# DeepSeek-R1

:::note
Demonstration Environment
:::

Demonstration Environment

Development Board : Jetson Orin series motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB requires the reduced-parameter version
:::

DeepSeek-R1 is an open-source Large Language Model (LLM) designed by DeepSeek for understanding and generating code.

## 1. Model Size

| Model | Size |
| --- | --- |
| deepseek-r1:1.5b | 1.1GB |
| deepseek-r1:7b | 4.7GB |
| deepseek-r1:8b | 5.2GB |

## 2. Performance

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-6/benchmark.jpg)

## 3. Using DeepSeek-R1

### 3.1 Running DeepSeek-R1

Use the run command to start running the R model. If the model is not downloaded, the model from the Ollama model library will be automatically downloaded:

```bash
ollama run deepseek-r1
```

```text
xxxxxxxxxx
```

```bash
ollama run deepseek-r1
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-6/1.png)

### 3.2 Starting a Conversation

```text
How many minutes are there in a day?
```

```text
xxxxxxxxxx
```

```text
How many minutes are there in a day?
```

Response time depends on your hardware configuration, so please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-6/image-20250628181905861.png)

### 3.3 Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

DeepSeek-R1

Ollama Model: https://ollama.com/library/deepseek-r1

GitHub: https://github.com/deepseek-ai/DeepSeek-r1
