---
title: 02-Write Jetson Original System
sidebar_position: 2
---

# Write Jetson original system

The tutorial demonstrates burning the NVIDIA official system image on the Jetson Orin motherboard. It is not recommended for beginners!

Note: 1. The solid-state drive that comes with the product has a factory image, which contains the NVIDIA official system and motherboard environment; if you do not require a pure system, do not operate!

2. The Orin nano SUB and Orin NX SUB series cannot activate the MAXN mode using this method. If the SUB board wants to use the MAXN mode, do not refer to this tutorial!

## 1. Hardware connection

### 1.1. Flashing mode

The Jetson Orin motherboard needs to short-circuit the FC REC and GND under the core board to enter the flashing mode:

![image-20250109180636160](/img/docs/hh101/02-Basics/2-2/image-20250109180636160.png)

### 1.2. Motherboard connection

The Jetson Orin motherboard needs to be connected to a DC power adapter, DP data cable, network cable and Type C data cable: Type C data cable connects to the computer

This illustration is based on the official version of Jetson Orin Nano. Users of other versions can refer to it for use (the hardware interface and functional layout are the same).

![image-20250109180831845](/img/docs/hh101/02-Basics/2-2/image-20250109180831845.png)

## 2. Software connection

The tutorial uses VMware Workstation 17 Pro to burn the system!

After starting the virtual machine, select in the menu bar: Virtual Machine → NVIDIA APX → Confirm the status of the connection with the virtual machine

![image-20250123100406992](/img/docs/hh101/02-Basics/2-2/image-20250123100406992.png)

## 3. Use of SDK Manager

To use SDK Manager, you need to register an account and log in in advance. Register and log in to the account by yourself!

```bash
You don't need to install SDK Manager to use our virtual machine. If you can't update SDK Manager, you can reinstall it according to the following tutorial
```

### 3.1, SDK Manager download

Official website: https://developer.nvidia.com/sdk-manager

![image-20250109171631722](/img/docs/hh101/02-Basics/2-2/image-20250109171631722.png)

### 3.2, SDK Manager installation

Enter the folder where the installation package is located and run the installation command:

```bash
sudo dpkg -i sdkmanager_2.2.0-12028_amd64.deb
```

Repair dependency: A clean system may have an error message when installing sdkmanager. You can run the following command and then rerun the installation command

```bash
sudo apt --fix-broken install -y
```

![image-20250109173336117](/img/docs/hh101/02-Basics/2-2/image-20250109173336117.png)

### 3.3, Start and log in to your account

Find the SDK Manager application icon in the system application menu bar or enter the following command in the terminal to start:

```bash
sdkmanager# To burn the old version of Jetpack, you need to use the command: sdkmanager --archived-versions
```

After clicking login, the browser login page will pop up automatically:

![image-20250109173851025](/img/docs/hh101/02-Basics/2-2/09173851025.png)

Enter the previously registered account and password to enter the software:

![image-20250109174138690](/img/docs/hh101/02-Basics/2-2/image-20250109174138690.png)

### 3.4, Motherboard selection

Choose according to your own motherboard model: For the official kit, select Jetson Orin Nano [*GB developer kit version]

![image-20250109174226263](/img/docs/hh101/02-Basics/2-2/image-20250109174226263.png)

### 3.5, STEP1

Check the options below, confirm that they are consistent, and click CONTINUE:

![image-20250109174317901](/img/docs/hh101/02-Basics/2-2/image-20250116184501461-1737441307076-4.png)

### 3.6, STEP2

Check the options below, confirm that they are consistent, and click CONTINUE:

![image-20250109174508253](/img/docs/hh101/02-Basics/2-2/image-20250109174508253.png)

![image-20250109174519757](/img/docs/hh101/02-Basics/2-2/image-20250109174519757.png)

Enter the virtual machine password: HemiHex

![image-20250109174549168](/img/docs/hh101/02-Basics/2-2/image-20250109174549168.png)

![image-20250109174556721](/img/docs/hh101/02-Basics/2-2/image-20250109174556721.png)

### 3.7, STEP3

Wait for the system to download successfully:

![image-20250109174659266](/img/docs/hh101/02-Basics/2-2/image-20250109174659266.png)

![image-20250109174901170](/img/docs/hh101/02-Basics/2-2/image-20250109174901170.png)

Select the motherboard model, OEM configuration, and system storage medium:

![image-20250109175753447](/img/docs/hh101/02-Basics/2-2/image-20250109175753447.png)

The virtual machine automatically controls the connection and disconnection of the device during the whole process, but more than 95% of the progress needs to pay attention to the SDK Manager burning system progress, once disconnected and not automatically connected to the virtual machine, you need to manually connect the motherboard to the virtual machine in time:

![image-20250109181833561](/img/docs/hh101/02-Basics/2-2/image-20250109181833561.png)

![image-20250116113445820](/img/docs/hh101/02-Basics/2-2/image-20250116113445820.png)

### 3.8, STEP4

![image-20250109182517227](/img/docs/hh101/02-Basics/2-2/image-20250109182517227.png)

## 4, start the system

After successfully burning the system using SDK Manager, you can disconnect the hardware connection between the motherboard and the computer and restore the motherboard to normal startup mode.

### 4.1, Normal mode

Unplug the FC REC and GND jumper caps that are shorted when flashing the Jetson Orin motherboard.

### 4.2, Start the system

Connect the DP data cable, network cable and Type C data cable to the Jetson motherboard, and finally use the DC power adapter to power the Jetson motherboard.

```bash
The Jetson Orin series motherboard needs to be connected to a monitor to display the screen normally and perform VNC remote
```

### 4.3, System settings

After starting the system, users can set system options according to the prompts of the system.

### Frequently asked questions

#### Unrecognized device

You can disconnect the DC power supply and Type C data cable of the motherboard, and then re-power it on to connect the virtual machine!
