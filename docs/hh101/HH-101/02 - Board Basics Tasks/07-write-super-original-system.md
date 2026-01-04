---
title: Write SUPER Original System
sidebar_position: 0
---

# Write SUPER original system

This tutorial supports upgrading SUPER from Jetson Orin official kit and Jetson Orin SUB kit. After upgrading SUPER, only the pure system will be retained, and the tutorial motherboard case cannot run.

:::note
The startup system of Jetson series motherboards is closely related to the Jetpack version of the motherboard. Different Jetpack versions may fail to start
:::

```
The tutorial uses VMware to start the Ubuntu22.04 virtual machine as a demonstration
```

```
The tutorial uses VMware to start the Ubuntu22.04 virtual machine as a demonstration
```

## 1. File download

Official website: https://developer.nvidia.com/embedded/jetson-linux-r3643

:::note
NVIDIA Jetson Linux 36.4.3 corresponds to Jetpack 6.2
:::

Download the compressed package files corresponding to Driver Package (BSP) and Sample Root Filesystem :

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120150532008.png)

## 2. Flashing mode

### 2.1. Hardware connection

Use a jumper cap to short-circuit the FC REC and GND pins under the core board: the core board can be left unassembled, the picture is just for a clearer observation

The Jetson Orin motherboard needs to be connected to a DC power adapter, DP data cable, network cable, and Type C data cable: Type C data cable connects to the computer

This illustration is based on the official version of Jetson Orin Nano. Users of other versions can refer to it for use (the hardware interface and functional layout are the same).

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120171038907.png)

### 2.2, Software connection

The motherboard is successfully connected to the Ubuntu system, and the lsusb command will show the NVIDIA Corp. APX information:

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120154019235.png)

## 3. Burning system

### 3.1. Unzip files

Go to the download folder and open the terminal, then unzip the file in the terminal and go to the specified folder:

```bash
tar xf Jetson_Linux_R36.4.3_aarch64.tbz2
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
cd Linux_for_Tegra/
```

```bash
tar xf Jetson_Linux_R36.4.3_aarch64.tbz2
```

```bash
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
```

```bash
cd Linux_for_Tegra/
```

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120151802071.png)

### 3.2. Run the script

```bash
sudo ./tools/l4t_flash_prerequisites.sh
```

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120151922577.png)

```bash
sudo ./apply_binaries.sh
```


![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120152138271.png)

### 3.3. Burn the system to the solid state drive

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 -c tools/kernel_flash/flash_l4t_t234_nvme.xml -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" --showlogs --network usb0 jetson-orin-nano-devkit-super internal
```

:::note
Both Jetson Orin Nano and Jetson Orin NX can use the same command to burn the system to the SSD
:::

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120152242874.png)

:::note
During the burning process, the user needs to connect the device to the virtual machine in time, otherwise it will cause the link to time out!
:::

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120155459747.png)

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120155405246.png)

## 4. Start the system

After the system is successfully burned, disconnect the mainboard power supply (disconnect the DC power adapter and Type-C data cable), and then unplug the jumper cap that shorts FC REC and GND under the core board; after confirming that the above operations are completed, reconnect the DC power adapter and DP data cable (connect to the display) to start the system.

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120161524122.png)

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120162024278-1737361225177-3.png)

![Screenshot](/img/docs/jetson/02-Basics/2-7/image-20250120162037749.png)

## 5. Component environment

The above is only to complete the burning of the pure system. If you need CUDA , TensorRT and other environments, users also need to refer to [Chapter 2 Motherboard Basics: Installing Jetson Component Environment] for operation!
