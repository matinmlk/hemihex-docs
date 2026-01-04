---
title: OpenRouter Large Model API Aggregation Platform
sidebar_position: 1
---

# 1.OpenRouter large model API aggregation platform

## Concept Introduction

### 1.1 What is "OpenRouter"?

OpenRouter is a large model API aggregation platform . You can think of it as a "universal remote control for large models." Typically, if you want to use OpenAI's GPT-4, Anthropic's Claude-3, and Google's Gemini, you need to register accounts on each platform, obtain three sets of API keys, and write three different sets of calling code for each of their different API formats.

OpenRouter solves this problem. It provides a unified API interface (compatible with OpenAI's API format). With just one OpenRouter API key, you can access all of these models, as well as hundreds of other open and closed-source models, through this single entry point.

The core advantages of integrating OpenRouter into largemodel projects are:

In the largemodel project, configure the llm_platform parameter to openrouter to enable this powerful aggregation platform.

## 2. Practical Operations

### 2.1 Obtaining API Credentials

### 2.2 Graphical Process

OpenRouter

![image-20250801163705985](/img/docs/jetson/12-OnlineAIModel/12-1/open.png)

![image-20250801165400079](/img/docs/jetson/12-OnlineAIModel/12-1/open router.png)

![image-20250807175353717](/img/docs/jetson/12-OnlineAIModel/12-1/image-20250807175353717.png)

![image-20250807175436229](/img/docs/jetson/12-OnlineAIModel/12-1/image-20250807175436229.png)

![image-20250807180439598](/img/docs/jetson/12-OnlineAIModel/12-1/image-20250807180439598.png)

![image-20250807180556227](/img/docs/jetson/12-OnlineAIModel/12-1/image-20250807180556227.png)

![image-20250801171806192](/img/docs/jetson/12-OnlineAIModel/12-1/api.png)

### 2.3 Configuration File Modifications

```bash
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

```bash
vim
~/yahboom_ws/src/largemodel/config/large_model_interface.yaml
```

In openrouter_api_key, replace sk-xxxxxxxxxx with the API key you copied earlier.

openrouter_model is used to configure the large model for communication.

```bash
xxxxxxxxxx
# OpenRouter Platform Configuration
openrouter_api_key:
"sk-xxxxxxxxxxxxxxxxxxx"
openrouter_model:
"moonshotai/kimi-k2:free"
# Model to use, for example, "google/gemini-pro-vision"
```

```bash
xxxxxxxxxx
```

```bash
# OpenRouter Platform Configuration
```

```bash
openrouter_api_key:
"sk-xxxxxxxxxxxxxxxxxxx"
```

```bash
openrouter_model:
"moonshotai/kimi-k2:free"
# Model to use, for example, "google/gemini-pro-vision"
```

```bash
xxxxxxxxxx
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

```bash
xxxxxxxxxx
```

```bash
vim
~/yahboom_ws/src/largemodel/config/HemiHex.yaml
```

```yaml
xxxxxxxxxx
llm_platform
:
'openrouter'
```

```bash
xxxxxxxxxx
```

```yaml
llm_platform
:
'openrouter'
```

After switching the platform parameters, return to the workspace and compile again for the changes to take effect.
