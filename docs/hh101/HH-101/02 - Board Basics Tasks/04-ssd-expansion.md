---
title: SSD Expansion
sidebar_position: 0
---

# SSD expansion

SSD expansion 1. Install GParted 2. Use GParted 3. Adjust partitions

The factory image system will perform disk compression, so the capacity displayed in the system will be inconsistent with the actual capacity. Users can follow the tutorial to expand the SSD.

```
The tutorial is located in the Jetson Orin motherboard system
```

```
The tutorial is located in the Jetson Orin motherboard system
```

## 1. Install GParted

```
xxxxxxxxxx
sudo apt update
sudo apt install gparted -y
```

```
xxxxxxxxxx
```

```bash
sudo apt update
```

```bash
sudo apt install gparted -y
```

![image-20250110203835949](/img/docs/jetson/02-Basics/2-4/image-20250110203835949.png)

## 2. Use GParted

Find the GParted application icon in the system application menu bar to open it or enter the following command in the terminal to start it:

```
xxxxxxxxxx
gparted
```

```
xxxxxxxxxx
```

```bash
gparted
```

![image-20250110203949546](/img/docs/jetson/02-Basics/2-4/image-20250110203949546.png)

## 3. Adjust partitions

Right-click the disk partition that needs to be expanded: generally select the largest partition in the disk

![image-20250110203959602](/img/docs/jetson/02-Basics/2-4/image-20250110203959602.png)

You can adjust the partition size through the slider: you can maximize the space and slide to the far right

![image-20250110204022401](/img/docs/jetson/02-Basics/2-4/image-20250110204022401.png)

Confirm the partition adjustment operation:

<!-- /![image-20250110204116310](/img/docs/jetson/02-Basics/2-4/image-20250110204116310.png) -->

![image-20250110204133498](/img/docs/jetson/02-Basics/2-4/image-20250110204133498.png)

![image-20250110204146422](/img/docs/jetson/02-Basics/2-4/image-20250110204146422.png)

![image-20250110204155043](/img/docs/jetson/02-Basics/2-4/image-20250110204155043.png)

After partitioning is completed, close the GParted software by yourself!
