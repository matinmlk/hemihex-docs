---
title: Use Jupyter Lab
sidebar_position: 0
---

# Use Jupyter Lab

Use Jupyter Lab 1. Jupyter Lab installation 1.1. Jupyter Lab 1.2、Node.js 2. Jupyter Lab startup 2.1. Set the default browser 2.2. Start Jupyter Lab 2.3. Host access 3. Jupyter Lab configuration 3.1. LAN access 3.1.1, create a configuration file 3.1.2, modify the configuration file 3.2, Configure access password 3.3, Start the service automatically at boot 3.3.1, Edit the service file 3.3.2, set up the self-start service Startup service automatically Start service Check service status Verify startup 4. Use Jupyter Lab 4.1. Kernel 4.2. Run the program 4.2.1. Running 4.2.2. Running completed

## 1. Jupyter Lab installation

### 1.1. Jupyter Lab

Use the following command to install Jupyter Lab: If the download speed of Jupyter Lab is slow, you can use the specified source to install it

```bash
sudo apt update
sudo apt install python3-pip -y
sudo pip3 install --upgrade pip
```

```bash
sudo apt update
```

```bash
sudo apt install python3-pip -y
```

```bash
sudo pip3 install --upgrade pip
```

```bash
xxxxxxxxxx
sudo pip3 install jupyterlab
# Tsinghua source: pip3 install jupyterlab -i https://pypi.tuna.tsinghua.edu.cn/simple
# Alibaba Cloud source: sudo pip3 install jupyterlab -i https://mirrors.aliyun.com/pypi/simple/
```

```bash
xxxxxxxxxx
```

```bash
sudo pip3 install jupyterlab
```

```bash
# Tsinghua source: pip3 install jupyterlab -i https://pypi.tuna.tsinghua.edu.cn/simple
```

```bash
# Alibaba Cloud source: sudo pip3 install jupyterlab -i https://mirrors.aliyun.com/pypi/simple/
```

![image-20241226141045667](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226141045667.png)

![image-20241226142651775](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226142651775.png)

### 1.2、Node.js

Use the following command to install the latest Node.js:

```bash
xxxxxxxxxx
sudo apt install curl -y
```

```bash
xxxxxxxxxx
```

```bash
sudo apt install curl -y
```

```bash
xxxxxxxxxx
sudo curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
```

```bash
xxxxxxxxxx
```

```bash
sudo curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
```

```bash
xxxxxxxxxx
sudo apt install nodejs -y
```

```bash
xxxxxxxxxx
```

```bash
sudo apt install nodejs -y
```

Verify the version:

```bash
xxxxxxxxxx
node -v && npm -v
```

```bash
xxxxxxxxxx
```

```bash
node -v && npm -v
```

![image-20241230104859936](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241230104859936.png)

## 2. Jupyter Lab startup

Before starting Jupyter Lab, you need to set the system default browser, otherwise some prompts will appear when starting the terminal.

### 2.1. Set the default browser

Open the system Chromium browser and select Set the default browser:

![image-20241226141850048](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226141850048.png)

![image-20241226141855084](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226141855084.png)

### 2.2. Start Jupyter Lab

```bash
xxxxxxxxxx
jupyter lab
# Start without browser jupyter lab --no-browser
# Start as administrator sudo jupyter lab --allow-root
```

```bash
xxxxxxxxxx
```

```bash
jupyter lab
```

```bash
# Start without browser jupyter lab --no-browser
```

```bash
# Start as administrator sudo jupyter lab --allow-root
```

![image-20241226143316322](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226143316322.png)

![image-20241226143326168](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226143326168.png)

### 2.3. Host access

The host refers to the Jetson motherboard system access, which can be accessed directly through http://localhost:8888/ :

```bash
xxxxxxxxxx
http://localhost:8888/
```

```bash
xxxxxxxxxx
```

```bash
http://localhost:8888/
```

![image-20250113180325457](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20250113180325457.png)

## 3. Jupyter Lab configuration

Configure LAN access, access password, and auto-start for Jupyter Lab.

### 3.1. LAN access

Device in the same LAN can be accessed by entering IP:8888 in the browser!

:::note
The LAN of the campus network is generally inaccessible. You can change the laptop/mobile phone hotspot to test
:::

```bash
xxxxxxxxxx
For example, the motherboard IP: 192.168.2.114; we can enter 192.168.2.114:8888 through the browser in the same LAN to perform Jupyter Lab on the motherboard
```

```bash
xxxxxxxxxx
```

```bash
For example, the motherboard IP: 192.168.2.114; we can enter 192.168.2.114:8888 through the browser in the same LAN to perform Jupyter Lab on the motherboard
```

#### 3.1.1, create a configuration file

```bash
xxxxxxxxxx
sudo jupyter lab --generate-config
```

```bash
xxxxxxxxxx
```

```bash
sudo jupyter lab --generate-config
```

The location of the automatically generated configuration file: Writing default config to: /root/.jupyter/jupyter_lab_config.py

#### 3.1.2, modify the configuration file

```bash
xxxxxxxxxx
sudo gedit /root/.jupyter/jupyter_lab_config.py
```

```bash
xxxxxxxxxx
```

```bash
sudo gedit /root/.jupyter/jupyter_lab_config.py
```

Modified content: After modification, click Save and close the file

```bash
xxxxxxxxxx
# Allow requests from any source to access the Jupyter Lab server
c.ServerApp.allow_origin = '*'
# 0.0.0.0 means binding all available network interfaces and allowing access from any address
c.ServerApp.ip = '0.0.0.0'
# Allow Jupyter Lab server to be started as root user
c.ServerApp.allow_root = True
# Modify the default port to avoid conflicts
c.ServerApp.port = 8888
```

```bash
xxxxxxxxxx
```

```bash
# Allow requests from any source to access the Jupyter Lab server
```

```bash
c.ServerApp.allow_origin = '*'
```

```bash
# 0.0.0.0 means binding all available network interfaces and allowing access from any address
```

```bash
c.ServerApp.ip = '0.0.0.0'
```

```bash
# Allow Jupyter Lab server to be started as root user
```

```bash
c.ServerApp.allow_root = True
```

```bash
# Modify the default port to avoid conflicts
```

```bash
c.ServerApp.port = 8888
```

![image-20241226144141750](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226144141750.png)

### 3.2, Configure access password

Enter the command to set the password in the terminal twice, and the input will not be displayed when entering the password!

```bash
xxxxxxxxxx
sudo jupyter lab password
```

```bash
xxxxxxxxxx
```

```bash
sudo jupyter lab password
```

Automatically generated configuration file location: [JupyterPasswordApp] Wrote hashed password to /root/.jupyter/jupyter_server_config.json

![image-20241226144212497](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226144212497.png)

### 3.3, Start the service automatically at boot

#### 3.3.1, Edit the service file

```bash
xxxxxxxxxx
sudo gedit /etc/systemd/system/jupyterlab.service
```

```bash
xxxxxxxxxx
```

```bash
sudo gedit /etc/systemd/system/jupyterlab.service
```

Add content: Click Save and close the file after adding

```bash
xxxxxxxxxx
[Unit]
Description=jupyterlab
After=network.target
[Service]
Type=simple
ExecStart=/usr/local/bin/jupyter-lab
config=/root/.jupyter/jupyter_lab_config.py --no-browser
User=root
Group=root
WorkingDirectory=/home/jetson/
Restart=always
RestartSec=10
[Install]
WantedBy=multi-user.target
```

```bash
xxxxxxxxxx
```

```bash
[Unit]
```

```bash
Description=jupyterlab
```

```bash
After=network.target
```

```bash
[Service]
```

```bash
Type=simple
```

```bash
ExecStart=/usr/local/bin/jupyter-lab
```

```bash
config=/root/.jupyter/jupyter_lab_config.py --no-browser
```

```bash
User=root
```

```bash
Group=root
```

```bash
WorkingDirectory=/home/jetson/
```

```bash
Restart=always
```

```bash
RestartSec=10
```

```bash
[Install]
```

```bash
WantedBy=multi-user.target
```

root: system user name

ExecStart: command to start Jupyter lab, change to JupyterLab installation path

config: change to JupyterLab configuration file path

WorkingDirectory: the working directory opened by starting Jupyter-lab, which can be changed by yourself (it is recommended to change to the user directory)

```bash
xxxxxxxxxx
Check Jupyter-lab installation path: which jupyter-lab
Configuration file path: refer to the path of the configuration file generated above
```

```bash
xxxxxxxxxx
```

```bash
Check Jupyter-lab installation path: which jupyter-lab
```

```bash
Configuration file path: refer to the path of the configuration file generated above
```

![image-20241226144326742](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226144326742.png)

#### 3.3.2, set up the self-start service

##### Startup service automatically

```bash
xxxxxxxxxx
sudo systemctl enable jupyterlab
# Disable startup systemctl disable jupyterlab
```

```bash
xxxxxxxxxx
```

```bash
sudo systemctl enable jupyterlab
```

```bash
# Disable startup systemctl disable jupyterlab
```

##### Start service

```bash
xxxxxxxxxx
sudo systemctl start jupyterlab
# Stop service sudo systemctl stop jupyterlab
```

```bash
xxxxxxxxxx
```

```bash
sudo systemctl start jupyterlab
```

```bash
# Stop service sudo systemctl stop jupyterlab
```

##### Check service status

```bash
xxxxxxxxxx
systemctl status jupyterlab
```

```bash
xxxxxxxxxx
```

```bash
systemctl status jupyterlab
```

![image-20241226144733028](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226144733028.png)

##### Verify startup

After restarting the system, use the same LAN device to access the motherboard IP:8888 based on the system IP.

```bash
xxxxxxxxxx
You need to enter a password for the first access, which is the information set in the previous step;
When taking the screenshot, the IP of the motherboard is 192.168.2.114, so devices in the same LAN can access 192.168.2.114:8888
```

```bash
xxxxxxxxxx
```

```bash
You need to enter a password for the first access, which is the information set in the previous step;
```

```bash
When taking the screenshot, the IP of the motherboard is 192.168.2.114, so devices in the same LAN can access 192.168.2.114:8888
```

![image-20241226144927314](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20241226144927314.png)

## 4. Use Jupyter Lab

### 4.1. Kernel

It is recommended to restart the kernel and clear all unit block output information every time you run a program or the program is abnormal:

![image-20250113180443385](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20250113180443385.png)

### 4.2. Run the program

Through Jupyter Open the program file to be run in Lab, and run the program from top to bottom to run the unit blocks in sequence:

#### 4.2.1. Running

[*] is displayed in the upper left corner of the unit block to indicate that it is running:

![image-20250113180823057](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20250113180823057.png)

#### 4.2.2. Running completed

[Number] is displayed in the upper left corner of the unit block to indicate the number of times it has been run: for example, [1] → the program has run the unit block code for the first time

![image-20250113180846984](/img/docs/jetson/05-VisionBasicCourse/5-3/image-20250113180846984.png)
