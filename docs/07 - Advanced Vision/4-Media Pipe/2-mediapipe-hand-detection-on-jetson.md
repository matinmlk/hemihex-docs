---
sidebar_position: 22
title: Hand Detection with MediaPipe
---

# Hand Detection

MediaPipe provides real-time **hand detection and tracking**,
identifying **21 key landmarks** per hand. This section demonstrates
hand detection on NVIDIA Jetson using **USB** and **CSI** cameras.

------------------------------------------------------------------------

## 1. Hand Detection Overview

MediaPipe Hand Detection can detect and track hands accurately in real
time, making it suitable for: - Gesture recognition - Human--computer
interaction - Robotics control - Vision-based interfaces

------------------------------------------------------------------------

## 2. USB Camera Hand Detection

### Navigate to MediaPipe Directory

``` bash
cd ~/mediapipe
```

### Run USB Camera Script

``` bash
python3 01.hand_usb.py
```

::: note
Click the preview window and press **q** to exit the program.
:::

------------------------------------------------------------------------

## 3. CSI Camera Hand Detection

### Navigate to MediaPipe Directory

``` bash
cd ~/mediapipe
```

### Run CSI Camera Script

``` bash
python3 01.hand_csi.py
```

::: note
Click the preview window and press **q** to exit the program.
:::

------------------------------------------------------------------------

## 4. Key Features

-   Detects **21 hand landmarks**
-   Real-time performance on Jetson
-   Supports **USB** and **CSI** cameras
-   Lightweight and efficient

------------------------------------------------------------------------

## Summary

-   MediaPipe enables accurate hand detection on Jetson
-   Works with both USB and CSI cameras
-   Ideal for interactive and control-based applications

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
