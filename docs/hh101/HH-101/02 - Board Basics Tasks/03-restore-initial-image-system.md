---
title: Restore Initial Image System
sidebar_position: 0
---

# Restore initial image system

## 1. Format the SSD

Before restoring the factory image, you need to format the SSD into exFAT format.

### 1.1. Download DiskGenius

Download URL:https://www.diskgenius.com/

![image-20250110201003552](/img/docs/jetson/02-Basics/2-3/image-20250123101304217.png)

Double-click the exe file you just downloaded to install DiskGenius. Follow the prompts to install the software on the Windows computer. After opening the software, it will be as shown below.

![image-20250123103324948](/img/docs/jetson/02-Basics/2-3/image-20250123103324948.png)

### 1.1. Use DiskGenius

#### 1.1.1, Delete partition

Deleting a partition will clear the disk data. Please confirm whether the drive letter is the disk that needs to be formatted before confirming the operation: you can judge based on the disk size and the newly added drive letter of the connected disk

![image-20250123103430264](/img/docs/jetson/02-Basics/2-3/image-20250123103430264.png)

![image-20250123103526303](/img/docs/jetson/02-Basics/2-3/image-20250123103526303.png)

![image-20250123103555176](/img/docs/jetson/02-Basics/2-3/image-20250123103555176.png)

![image-20250123103612549](/img/docs/jetson/02-Basics/2-3/image-20250123103612549.png)

#### 1.1.2. Create a new partition

Partition the SSD into NTFS format.

Select the drive letter corresponding to the SSD, and then click New Partition:

![image-20250123103732426](/img/docs/jetson/02-Basics/2-3/image-20250123103732426.png)

![image-20250123103803273](/img/docs/jetson/02-Basics/2-3/image-20250123103803273.png)

![image-20250123103855896](/img/docs/jetson/02-Basics/2-3/image-20250123103855896.png)

![image-202501231039252 21](/img/docs/jetson/02-Basics/2-3/image-20250123103925221.png)

![image-20250123103937125](/img/docs/jetson/02-Basics/2-3/image-20250123103937125.png)

![image-20250123103950870](/img/docs/jetson/02-Basics/2-3/image-20250123103950870.png)

![image-20250123104017947](/img/docs/jetson/02-Basics/2-3/image-20250123104017947.png)

## 2. Restore the factory image

You need to download and decompress the factory image system in the data to the local computer in advance.

### 2.1. Install Win32DiskImager

Download URL:https://sourceforge.net/projects/win32diskimager/

![image-20250110222239328](/img/docs/jetson/02-Basics/2-3/image-20250110222239328.png)

Open thewin32diskimager-1.0.0-install.exeinstallation package as an administrator and accept the agreement:

![image-20250110222209318](/img/docs/jetson/02-Basics/2-3/image-20250110222209318.png)

Installation location: The default location is recommended

![image-2025011022214163 9](/img/docs/jetson/02-Basics/2-3/image-20250110222141639.png)

Installation options:

![image-20250110222109847](/img/docs/jetson/02-Basics/2-3/image-20250110222109847.png)

![image-20250110222041855](/img/docs/jetson/02-Basics/2-3/image-20250110222041855.png)

Start installation:

![image-20250110222017810](/img/docs/jetson/02-Basics/2-3/image-20250110222017810.png)

Complete installation:

![image-20250110221950093](/img/docs/jetson/02-Basics/2-3/image-20250110221950093.png)

### 2.2. Use Win32DiskImager

①: Select the factory image file (*.img) in the data

②: Select the drive letter corresponding to the solid-state drive

③: Write the factory image to the solid-state drive

![image-20250123105549673](/img/docs/jetson/02-Basics/2-3/image-20250123105549673.png)

Confirm writing to the system:

![image-20250123105608261](/img/docs/jetson/02-Basics/2-3/image-2 0250123105608261.png)

Wait for the system to be written successfully:

![image-20250123105647187](/img/docs/jetson/02-Basics/2-3/image-20250123105647187.png)

![image-20250123111430147](/img/docs/jetson/02-Basics/2-3/image-20250123111430147.png)

After the system is written, you can close the program and install the SSD to the Jetson Orin motherboard!

## 3. Description

The Jetson motherboard can start the system normally and it depends on the system Jetpack version. Generally, only the same version can start the system!
