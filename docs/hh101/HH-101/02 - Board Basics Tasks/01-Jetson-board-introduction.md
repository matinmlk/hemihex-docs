---
title: 01-Jetson Board Introduction
sidebar_position: 1
---

# 01-Jetson Board Introduction

## 1. What Is a Jetson Board?

NVIDIA Jetson boards are compact edge AI computers. They run Linux, support GPU acceleration, and are built for robotics, vision, and AI inference workloads.

Compared with a microcontroller, Jetson can run full operating systems and AI frameworks. Compared with a desktop GPU PC, Jetson is lower power and designed for embedded deployment.

## 2. Why Jetson Orin Nano?

Jetson Orin Nano is a common entry point for:

- Learning CUDA and TensorRT on embedded hardware
- Building camera-based AI projects
- Running ROS/ROS 2 with onboard acceleration

## 3. Development Kit Variants

The main difference between the Jetson Orin official development kit and the SUB development kit is that the official kit does not include a power switch button.

### 3.1 Front of the Development Board

![Front view of Jetson Orin development board with labeled interfaces](/img/docs/hh101/02-Basics/2-1/image-20250109194932086.png)

| Serial Number | Description                          | Serial Number | Description                  |
|---------------|--------------------------------------|---------------|------------------------------|
| 1             | Power switch button (SUB)            | 9             | CAN bus                      |
| 2             | 12-pin button header                 | 10            | Fan interface                |
| 3             | Camera interface 1 (22-pin)          | 11            | 40-pin GPIO expansion header |
| 4             | Camera interface 2 (22-pin)          | 12            | Core module card holder      |
| 5             | PoE reverse power interface (1x2 pin)| 13            | PoE interface                |
| 6             | USB 3.0 x4                           | 14            | Ethernet interface           |
| 7             | DisplayPort interface                | 15            | Power indicator              |
| 8             | DC power interface                   | 16            | USB Type-C                   |

### 3.2 Back of the Development Board

![Back view of Jetson Orin development board showing M.2 slots](/img/docs/hh101/02-Basics/2-1/image-20250109194957501.png)

| Serial Number | Description                       | Serial Number | Description             |
|---------------|-----------------------------------|---------------|-------------------------|
| 1             | M.2 Key E connector slot (75-pin) | 3             | M.2 Key M slot (75-pin) |
| 2             | RTC battery holder (optional)     | 4             | M.2 Key M slot (75-pin) |

## 4. Software Stack You Will See in HH-101

In the next setup lessons you will work with:

- JetPack (NVIDIA software bundle for Jetson)
- Ubuntu on the target board
- SDK Manager on a host machine
- CUDA, TensorRT, and related runtime components
