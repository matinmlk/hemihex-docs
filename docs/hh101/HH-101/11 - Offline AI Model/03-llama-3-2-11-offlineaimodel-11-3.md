---
title: Llama 3.2
sidebar_position: 0
---

# Llama 3.2

Demo Environment

Development Board : Jetson Orin series motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB requires the reduced-parameter version
:::

Meta Llama 3.2 is a series of advanced open-source large-scale language models (LLMs) developed by the Meta AI department.

## 1. Model Size

| Model | Size |
| --- | --- |
| llama3.2:1b | 1.3GB |
| llama3.2:3b | 2.0GB |

## 2. Performance

![Llama 3.2 instruction-tuned benchmarks](/img/docs/jetson/11-OfflineAIModel/11-3/c1a51716-d8bb-4642-8044-48f5022b777d.png)

## 3. Using Llama 3.2

### 3.1 Running Llama 3.2

Use the run command to run the model. If the model is not already downloaded, it will automatically pull the model from the Olama model library:

```bash
ollama run llama3.2:3b
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-3/1.png)

### 3.2 Starting a Conversation

```bash
How many minutes are there in a day?
```

Response time depends on your hardware configuration, so please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-3/image-20250627185035604.png)

### 3.3 Ending a Conversation

Use the Ctrl+d shortcut or /bye to end a conversation!

## References

Ollama

Official Website: https://ollama.com/

GitHub: https://github.com/ollama/ollama

Llama 3.2

Official Website: https://www.llama.com/docs/model-cards-and-prompt-formats/llama3_2/

Ollama Model: https://ollama.com/library/llama3.2
