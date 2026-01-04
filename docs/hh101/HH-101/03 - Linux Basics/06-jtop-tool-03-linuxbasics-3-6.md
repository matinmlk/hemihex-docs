---
title: Jtop Tool
sidebar_position: 0
---

# Jtop tool

Jtop is a system monitoring tool developed for NVIDIA Jetson series devices. It can display the resource usage of various aspects of Jetson devices, such as CPU, GPU, memory, disk, network, etc., and can display different hardware temperatures, power consumption, frequency, etc. in real time.

## 1. Install Jtop

```bash
sudo apt update
sudo apt install python3-pip -y
sudo pip3 install -U jetson-stats
```

![image-20241228194459229](/img/docs/jetson/03-LinuxBasics/3-6/image-20241228194459229.png)

## 2. Best performance mode

### 2.2. Enable MAXN mode

Enabling MAXN Power Mode on Jetson will ensure that all CPU and GPU cores are turned on:

```bash
sudo nvpmodel -m 2
```

### 2.2. Enable Jetson Clocks

Enabling Jetson Clocks will ensure that all CPU and GPU cores run at maximum frequency:

```bash
sudo jetson_clocks
```


## 3. Use Jtop

Only after restarting the system can you enter the jtop command in the terminal to start the Jtop tool:

```bash
jtop
```


:::note
: The motherboard power mode must be set to MAXN to display the strongest performance parameters!
:::

![image-20250122195343553](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195343553.png)

![image-20250122195350543](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195350543.png)

![image-20250122195358919](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195358919.png)

![image-20250122195405567](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195405567.png)

![image-20250122195413447](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195413447.png)

![image-20250122195419958](/img/docs/jetson/03-LinuxBasics/3-6/image-20250122195419958.png)

![image-20250121150858152](/img/docs/jetson/03-LinuxBasics/3-6/image-20250121150858152.png)
