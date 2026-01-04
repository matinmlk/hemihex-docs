---
title: Phi4-mini
sidebar_position: 0
---

# Phi4-mini

Demonstration Environment

Development Board : Jetson Orin Series Board

:::warning
Due to performance limitations, this model cannot be run on the Jetson Orin Nano 4GB.
:::

The Phi-4-mini-instruction is a lightweight, open model built on synthetic data and curated public websites, focusing on high-quality, inference-intensive data.

## 1. Model Scale

| Model | Volume |
| --- | --- |
| phi4-mini:3.8b | 2.5GB |

## 2. Performance

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-5/F4.webp)

## 3. Using Phi4-mini

### 3.1 Running Phi4-mini

Use the run command to start running the model. If the model has not been downloaded, it will automatically pull the model from the Ollama model library:

```bash
ollama run phi4-mini:3.8b
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-5/1.png)

### 3.2 Starting a Conversation

```bash
How many minutes is a quarter of an hour?
```


Response time depends on your hardware configuration. Please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-5/image-20250628174758777.png)

### 3.3 Ending a Conversation

Use the Ctrl+D shortcut or /bye to end a conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

Phi4-mini

Ollama Compatible Model: https://ollama.com/library/phi4-mini
