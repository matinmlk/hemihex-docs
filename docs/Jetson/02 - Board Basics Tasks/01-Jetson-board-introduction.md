---
title: Jetson Board Introduction
sidebar_position: 0
---

# Jetson Board Introduction

## 1. Jetson Orin Development Kit

The main difference between the Jetson Orin **official development kit** and the **SUB version development kit** is that the official kit does not include the **power switch button (①)**.

### 1.1 Front of the Development Board

![Front view of Jetson Orin development board with labeled interfaces](/img/docs/jetson/02-Basics/2-1/image-20250109194932086.png)

| Serial Number | Description                                   | Serial Number | Description                              |
|---------------|-----------------------------------------------|---------------|------------------------------------------|
| 1             | Power switch button (SUB)                     | 9             | CAN bus                                  |
| 2             | 12-pin button header                          | 10            | Fan interface                            |
| 3             | Camera interface 1 (22-pin)                   | 11            | 40-pin GPIO expansion header             |
| 4             | Camera interface 2 (22-pin)                   | 12            | Core module card holder                  |
| 5             | PoE reverse power supply interface (1×2 pin) | 13            | PoE interface                            |
| 6             | USB 3.0 ×4                                   | 14            | Ethernet interface                       |
| 7             | DisplayPort interface                        | 15            | Power indicator                          |
| 8             | Power interface                               | 16            | USB Type-C                               |

### 1.2 Back of the Development Board

![Back view of Jetson Orin development board showing M.2 slots](/img/docs/jetson/02-Basics/2-1/image-20250109194957501.png)

| Serial Number | Description                       | Serial Number | Description             |
|---------------|-----------------------------------|---------------|-------------------------|
| 1             | M.2 Key E connector slot (75-pin) | 3             | M.2 Key M slot (75-pin) |
| 2             | RTC battery holder (optional)     | 4             | M.2 Key M slot (75-pin) |
