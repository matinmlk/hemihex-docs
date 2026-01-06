---
title: Qwen3
sidebar_position: 0
---

# Qwen3

Demo Environment

Development Board : Jetson Orin Series Motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB must run the reduced-parameter version
:::

Qwen 3 is the latest generation of large-scale language models in the Qwen series, providing a complete suite of dense and mixture-of-experts (MoE) models.

## 1. Model Size

| Model | Size |
| --- | --- |
| qwen3:0.6b | 523MB |
| qwen3:1.7b | 1.4GB |
| qwen3:4b | 2.6GB |
| qwen3:8b | 5.2GB |

## 2. Performance

![Evaluation 1](/img/docs/jetson/11-OfflineAIModel/11-4/37a622f9-71bb-49df-9fea-161b925e4853.png)

![Evaluation 2](/img/docs/jetson/11-OfflineAIModel/11-4/7ac69e9c-c4d9-45c1-8bd6-a9787c8c454c.png)

## 3. Using Qwen3

### 3.1. Running Qwen3

Use the run command to start running the model. If you haven't downloaded the model, the model from the Ollama model library will be automatically downloaded:

```bash
ollama run qwen3:8b
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-4/1.png)

### 3.2. Starting a Conversation

```bash
Please tell me how many hours there are in a day.
```


Response time depends on your hardware configuration, so please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-4/image-20250628112014942.png)

### 3.3. Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

Qwen3

GitHub: https://github.com/QwenLM/Qwen3

Ollama Model: https://ollama.com/library/qwen3
