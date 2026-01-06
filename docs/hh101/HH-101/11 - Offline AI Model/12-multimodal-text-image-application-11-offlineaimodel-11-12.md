---
title: Multimodal Text Image Application
sidebar_position: 0
---

# Multimodal Text Image Application

Since Ollama doesn't support text-to-image, we need to use other tools to implement this functionality natively.

## 1. Concept Introduction

### 1.1 What is Text-to-Image?

Text-to-Image is an AI technology that automatically generates images based on text descriptions . Simply enter a text (e.g., "A Shiba Inu wearing sunglasses surfing on the beach"), and the AI ​​model will generate an image that matches the description based on semantic understanding. No painting or design skills are required.

#### Core Principles

### 1.2 What is FastSDCPU?

FastSDCPU is an open-source Stable Diffusion image processing project optimized for CPU devices. Through algorithmic and engineering optimizations, it enables rapid generation of high-quality images on standard computers without a GPU, significantly lowering the hardware barrier to entry for AI painting.

#### Core Features

#### Applicable Scenarios

## 2. Project Deployment

### 2.1 Deployment Environment

:::warning
If using our pre-installed image, there is no need to deploy an environment and you can skip the deployment steps. Simply refer to [2.4 How to Start After Deployment] at the bottom to start the project directly.
:::

Open a terminal and execute the following code:

```bash
# If Git is not installed on your motherboard, run it first.
​
sudo apt update
sudo apt install git -y
sudo apt install python3.10-venv -y
​
# Add environment variables
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
​
# Clone the project code
git clone https://github.com/rupeshs/fastsdcpu.git
cd fastsdcpu
​
# Create a virtual environment and install dependencies
python -m venv venv
source venv/bin/activate
​
# Install uv
curl -Ls https://astral.sh/uv/install.sh | sh (This step may not work if you don't have a proxy in China. If not, skip this step and proceed to the next command.)
​
# If installing uv using curl in the previous step fails, run these three commands.
wget https://mirrors.huaweicloud.com/astral/uv/0.8.4/uv-aarch64-unknown-linux-gnu -O ~/.local/bin/uv
chmod +x ~/.local/bin/uv
~/.local/bin/uv --version
​
chmod +x install.sh start-webui.sh
./install.sh --disable-gui
```


Installation successful, press any key to exit:

![image-20250801214649785](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250801214649785.png)

### 2.2 LAN Access

Before starting, you need to modify a file to support LAN access. Otherwise, the webui can only be accessed locally:

```bash

vim ~/fastsdcpu/src/frontend/webui/ui.py
```


```bash
vim ~/fastsdcpu/src/frontend/webui/ui.py
```

After opening the ui.py file, scroll to the last line and find Change the line webui.launch(share=share) to webui.launch(server_name="0.0.0.0",share=share)

Save the code.

Start:

```bash

./start-webui.sh
```


```bash
./start-webui.sh
```

![image-20250801221311739](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250801221311739.png)

You can then access the webui by entering your motherboard's IP address: 7860 in your browser.

### 2.3 Using the Vinyl Image Function

Use ifconfig in the terminal to query your motherboard's IP address. For example, mine is 192.168.2.106.

Then open your browser and enter your motherboard's IP address: 7860 . For example, I entered 192.168.2.106:7860, and you'll be able to access the webui.

![image-20250804105134704](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804105134704.png)

Next, click LCM-LoRA. This model uses less memory, but if you'd like to use a different model, feel free to research it yourself.

Next, click Models to see the LCM-LoRA model settings. You can change the model to your preference, or just stick with the default like I did.

![image-20250804105439656](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804105439656.png)

Next, click Generation Settings and increase the Inference Steps setting to improve the quality of the generated image. I've set it to 5.

![image-20250804111046973](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804111046973.png)

Next, return to the Text to Image dialog box and enter the content you want to generate. Click Generate to begin generating the image.

![image-20250804105656393](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804105656393.png)

For first-time users, you'll need to download the model. You'll see the default model being downloaded in the terminal. Once it's finished downloading, the text-to-image function will begin.

![image-20250804105710057](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804105710057.png)

Generated result:

![image-20250804113118977](/img/docs/jetson/11-OfflineAIModel/11-12/image-20250804113118977.png)

This project has better support for English, and the generated images are more consistent with the text. We recommend using English descriptions when generating images.

### 2.4 How to start after successful deployment

```bash
cd fastsdcpu #Enter the fastsdcpu directory
source venv/bin/activate #Enter the virtual environment
./start-webui.sh #Start the webui
```


After the webui is successfully started, enter your motherboard's IP address: 7860 in your browser to start the image generation function.
