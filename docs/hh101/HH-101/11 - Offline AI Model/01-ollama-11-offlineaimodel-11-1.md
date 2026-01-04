---
title: Ollama
sidebar_position: 0
---

# Ollama

Ollama 1. Large Language Model (LLM) 2. Ollama Installation Script Installation 3. Use Ollama 4. Ollama Uninstall References

> Demo Environment

Demo Environment

Development board : Jetson Orin series motherboard

Ollama is an open source tool that aims to simplify the deployment and operation of large language models, allowing users to use high-quality language models in local environments.

## 1. Large Language Model (LLM)

Large Language Models (LLM) are a type of advanced text generation system based on artificial intelligence technology. Its main feature is that it can learn and understand human language through large-scale training data and can generate natural and fluent text.

## 2. Ollama Installation

The tutorial demonstrates the use of scripts to install Ollama on the Jetson Orin series motherboard.

### Script Installation

```bash
xxxxxxxxxx
sudo apt install curl -y
sudo curl -fsSL https://ollama.com/install.sh | sh
```

```bash
xxxxxxxxxx
```

```bash
sudo apt install curl -y
```

```bash
sudo curl -fsSL https://ollama.com/install.sh | sh
```

![image-20250111132928934](/img/docs/jetson/11-OfflineAIModel/11-1/image-20250111132928934.png)

The entire installation process takes a long time, please wait patiently!

## 3. Use Ollama

Type ollama in the terminal and you will see the prompt:

![image-20250111135927014](/img/docs/jetson/11-OfflineAIModel/11-1/image-20250111135927014.png)

| Command | Purpose |
| --- | --- |
| ollama serve | Start ollama |
| ollama create | Create a model from a model file |
| ollama show | Show model information |
| ollama run | Run a model |
| ollama pull | Pull a model from a registry |
| ollama push | Push a model to a registry |
| ollama list | List models |
| ollama ps | List running models |
| ollama cp | Copy a model |
| ollama rm | Delete a model |
| ollama help | Get help information about any command |

## 4. Ollama Uninstall

```bash
xxxxxxxxxx
sudo systemctl stop ollama
sudo systemctl disable ollama
sudo rm /etc/systemd/system/ollama.service
```

```bash
xxxxxxxxxx
```

```bash
sudo systemctl stop ollama
```

```bash
sudo systemctl disable ollama
```

```bash
sudo rm /etc/systemd/system/ollama.service
```

```bash
xxxxxxxxxx
sudo rm $(which ollama)
```

```bash
xxxxxxxxxx
```

```bash
sudo rm $(which ollama)
```

```bash
xxxxxxxxxx
sudo rm -r /usr/share/ollama
sudo userdel ollama
sudo groupdel ollama
```

```bash
xxxxxxxxxx
```

```bash
sudo rm -r /usr/share/ollama
```

```bash
sudo userdel ollama
```

```bash
sudo groupdel ollama
```

## References

> Ollama

Ollama

Official website: https://ollama.com/

GitHub: https://github.com/ollama/ollama
