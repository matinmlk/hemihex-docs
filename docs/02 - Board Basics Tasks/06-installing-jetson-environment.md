---
title: Installing Jetson environment
sidebar_position: 0
---

# Installing Jetson environment

Installing Jetson environment 1. Hardware connection 2. Software connection 3. Use of SDK Manager 3.1, Motherboard selection 3.2, STEP1 3.3, STEP2 3.4, STEP3 3.5、STEP4 4、Environment verification

Some users may need to use NVIDIA's own system component environment. After installing the system and successfully entering the system desktop, you can follow the tutorial below to install the component environment!

:::note
The solid-state drive that comes with the product has a factory image, which includes NVIDIA's official system and motherboard environment; if you do not have a requirement for a pure system, do not operate!
:::

```
The entire process requires a network. If the user cannot complete the component installation, use our factory image
```

```
The entire process requires a network. If the user cannot complete the component installation, use our factory image
```

## 1. Hardware connection

The Jetson Orin motherboard needs to be connected to a DC power adapter, DP data cable, network cable, and Type C data cable: The Type C data cable is connected to a computer or virtual machine

This illustration is based on the official version of Jetson Orin Nano. Users of other versions can refer to it for use (the hardware interface and functional layout are the same).

:::note
There is no need to enter the flashing mode to install the Jetson component environment, the motherboard can be turned on!
:::

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250109180831845.png)

## 2. Software connection

The tutorial uses VMware Workstation 17 Pro to burn the system!

After starting the virtual machine, select in the menu bar: Virtual Machine → NVIDIA Linux for Tegra → Confirm the status of the connection with the virtual machine

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250123100937979.png)

## 3. Use of SDK Manager

The Jetson Orin motherboard is in the normal system startup state, and the Type-C data cable is used to connect to the computer or virtual machine.

### 3.1, Motherboard selection

After opening `SDK Manager` , select according to your motherboard model: select `Jetson Orin Nano [*GB developer kit version]` for the official kit

### 3.2, STEP1

Check the options below, confirm that the check is consistent, and click `CONTINUE` :

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194316209-1737442070436-3.png)

### 3.3, STEP2

The system has been successfully installed, we only need to check the required component environment: SDK Manager will automatically download all components. Once the download is complete, you can proceed to the next step

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194450017.png)

Enter the virtual machine password: HemiHex

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194516622.png)

### 3.4, STEP3

Here, fill in the username and password information you set when starting the system: the whole process takes a long time, please wait patiently

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194558310.png)

Wait for the system to download and install: the virtual machine has a network failure during the whole process, which can be ignored. The actual test did not affect the burning of the component environment

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194622869.png)

The whole installation process takes a long time, please wait patiently. If the installation fails, you can restart the motherboard system and restart the SDK Manager installation:

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116194818632.png)

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116200418604.png)

### 3.5、STEP4

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250116203345930.png)

## 4、Environment verification

:::note
Jtop tool
:::

Jtop tool

Use the following command to install Jtop:

```bash
sudo apt update && sudo apt upgrade
sudo apt install python3-pip -y
sudo pip3 install -U jetson-stats
```

```
xxxxxxxxxx
```

```bash
sudo apt update && sudo apt upgrade
```

```bash
sudo apt install python3-pip -y
```

```bash
sudo pip3 install -U jetson-stats
```

:::note
After the installation is complete, you need to restart the system!
:::

:::note
Best Performance Mode
:::

Best Performance Mode

Enable MAX Power Mode

```bash
sudo nvpmodel -m 2 # Jetson Orin Nano
sudo nvpmodel -m 0 # Jetson Orin NX
```

```
xxxxxxxxxx
```

```bash
sudo nvpmodel -m 2 # Jetson Orin Nano
```

```bash
sudo nvpmodel -m 0 # Jetson Orin NX
```

![88d179920f811d989b1c909d6adb59b9](/img/docs/jetson/02-Basics/2-6/88d179920f811d989b1c909d6adb59b9.png)

Enable Jetson clocks: CPU and GPU cores run at maximum frequency

```bash
sudo jetson_clocks
```

```
xxxxxxxxxx
```

```bash
sudo jetson_clocks
```

Use the Jtop tool to view system information:

```bash
jtop
```

```
xxxxxxxxxx
```

```bash
jtop
```

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250121150457398.png)

![Screenshot](/img/docs/jetson/02-Basics/2-6/image-20250121150858152.png)
