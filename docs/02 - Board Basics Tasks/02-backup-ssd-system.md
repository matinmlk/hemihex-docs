---
title: Backup SSD System
sidebar_position: 0
---

# Backup SSD system

Backup SSD system1. Hardware connection2. Compress the SSD2.1. Install Gparted2.2. Use GParted2.2.1. Select the SSD2.2.2. Unmount the partition2.2.3. Perform disk compression3. Back up the SSD3.1. Check disk information3.2. Start disk backup

During the development process, users may need to back up the system to prevent subsequent development from affecting the current system environment.

## 1. Hardware connection

Users need to prepare the SSD box in advance, install the SSD into the SSD box and connect it to the computer or virtual machine: the computer and virtual machine systems need to be Ubuntu systems.

![image-20250110185143770](/img/docs/jetson/02-Basics/2-2/image-20250110185143770.png)

## 2. Compress the SSD

Since the SSD capacity of the Jetson Orin series motherboard is relatively large, we need to compress it to an appropriate space for system backup to save time for backup and burning the system.

### 2.1. Install Gparted

![image-20250110191004723](/img/docs/jetson/02-Basics/2-2/image-20250110191004723.png)

### 2.2. Use GParted

Find theGPartedapplication icon in the system application menu bar to open it or enter the following command in the terminal to start it:

![image-20250110191852838](/img/docs/jetson/02-Basics/2-2/image-20250110191852838.png)

Select the newly added disk symbol: You can confirm again whether it is the SSD you mounted based on the disk capacity

![image-20250110192029192](/img/docs/jetson/02-Basics/2-2/image-20250110192029192.png)

Before operating the disk, you need to unmount the disk: select theAPPpartition (largest partition) in the disk, and clickUnmountto unmount the partition

![image-20250110192528171](/img/docs/jetson/02-Basics/2-2/image-20250110192528171.png)

Right-click the uninstalled disk partition and resize the previously uninstalled partition space:

![image-20250110192743689](/img/docs/jetson/02-Basics/2-2/image-20250110192743689.png)

You can adjust the partition size using the slider: yellow is the space used by the partition, white is the unused space, it is recommended to leave about 5-10G of unused space in the partition to avoid the system from failing to start

![image-20250110192847118](/img/docs/jetson/02-Basics/2-2/image-20250110192847118.png)

Confirm the disk operation:

![image-20250 110192916523](/img/docs/jetson/02-Basics/2-2/image-20250110192916523.png)

![image-20250110193003482](/img/docs/jetson/02-Basics/2-2/image-20250110193003482.png)

Wait for the operation to complete:

![image-20250110193101217](/img/docs/jetson/02-Basics/2-2/image-20250110193101217.png)

![image-20250110194244239](/img/docs/jetson/02-Basics/2-2/image-20250110194244239.png)

After completing the above operations, close GParted!

## 3. Back up the SSD

### 3.1. Check disk information

Open the terminal and use the script to view the current disk information: the drive letter needs to correspond to the drive letter of the SSD you backed up

parted_info.sh script content

Record the data in the figure: 41822208s

![image-20250110194409241](/img/docs/jetson/02-Basics/2-2/image-20250110194409241.png)

### 3.2. Start disk backup

Use theddcommand to back up the SSD to theimgfile. Enter the following in the terminal:

/dev/sdb: SSD drive letter

Jetson_Orin_Nano_8G.img: Image name

bs=512: Set block size to 512 bytes

56393728: Data queried by the script

![image-20250110194525743](/img/docs/jetson/02-Basics/2-2/image-20250110194525743.png)

To view theddprocess information, open another terminal and enter the following command:

![image-20250110194614587](/img/docs/jetson/02-Basics/2-2/image-20250110194614587.png)

Wait for the backup to complete:

![image-20250110195846900](/img/docs/jetson/02-Basics/2-2/image-20250110195846900.png)

After the system backup is complete, move the backup file (Jetson_Orin_Nano_8G.img) to the Windows system for use.

![image-20250110195926598](/img/docs/jetson/02-Basics/2-2/image-20250110195926598.png)
