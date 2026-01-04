---
title: Gemma3
sidebar_position: 0
---

# Gemma3

:::note
Demo Environment
:::

Demo Environment

Development Board : Jetson Orin series motherboard

:::warning
Due to performance limitations, the Jetson Orin Nano 4GB requires the reduced-parameter version
:::

Gemma is a family of lightweight models built by Google based on Gemini technology. The Gemma 3 model is multimodal (capable of processing text and images), has a 128KB context window, and supports over 140 languages.

## 1. Model Size

| Model | Size |
| --- | --- |
| gemma3:1b | 815MB |
| gemma3:4b | 3.3GB |

## 2. Performance

![Chatbot Arena ELO Score](/img/docs/jetson/11-OfflineAIModel/11-8/89dc5a19-179e-4dd3-8e5d-12ad54973148.png)

## 3. Using Gemma3

### 3.1 Running Gemma3

Use the run command to run the model. If the model is not already downloaded, it will automatically pull the model from the Ollama model library:

```bash
ollama run gemma3:4b
```

```bash
xxxxxxxxxx
```

```bash
ollama run gemma3:4b
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-8/1.png)

### 3.2 Start a conversation

```bash
How to learn a programming language?
```

```bash
xxxxxxxxxx
```

```bash
How to learn a programming language?
```

Response time depends on hardware configuration, so please be patient!

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-8/image-20250630115641821.png)

### 3.3 Visual Function

![test_pic](/img/docs/jetson/11-OfflineAIModel/11-8/test_pic.png)

```bash
What do you see in this picture? :./test_pic.png
#Using ": + the image path" in the conversation allows the model to use its visual function and interpret the information in the image.
```

```bash
xxxxxxxxxx
```

```bash
What do you see in this picture? :./test_pic.png
```

```bash
#Using ": + the image path" in the conversation allows the model to use its visual function and interpret the information in the image.
```

![Screenshot](/img/docs/jetson/11-OfflineAIModel/11-8/3.png)

### 3.4 Ending the Conversation

Use the Ctrl+d shortcut or /bye to end the conversation!

## References
**Ollama**
Ollama
- **Official Website**: https://ollama.com/
- **GitHub**: https://github.com/ollama/ollama
**Gemma3**
Gemma3
- **Ollama Compatible Model**: https://ollama.com/library/gemma3
