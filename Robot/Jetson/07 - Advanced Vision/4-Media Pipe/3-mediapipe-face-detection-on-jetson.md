---
sidebar_position: 23
title: Face Detection with MediaPipe
---

# Face Detection

MediaPipe provides real-time **face detection** capabilities, enabling
fast and accurate identification of human faces in images and video
streams. This section demonstrates face detection on NVIDIA Jetson using
**USB** and **CSI** cameras.

------------------------------------------------------------------------

## 1. Face Detection Overview

MediaPipe Face Detection can: - Detect faces in real time - Output
bounding boxes and confidence scores - Run efficiently on embedded
devices

Typical applications include: - Human--computer interaction -
Intelligent surveillance - Face-based analytics

------------------------------------------------------------------------

## 2. USB Camera Face Detection

### Navigate to MediaPipe Directory

``` bash
cd ~/mediapipe
```

### Run USB Camera Script

``` bash
python3 02.face_detection_usb.py
```

::: note
Click the preview window and press **q** to exit the program.
:::

------------------------------------------------------------------------

## 3. CSI Camera Face Detection

### Navigate to MediaPipe Directory

``` bash
cd ~/mediapipe
```

### Run CSI Camera Script

``` bash
python3 02.face_detection_csi.py
```

::: note
Click the preview window and press **q** to exit the program.
:::

------------------------------------------------------------------------

## 4. Key Features

-   Real-time face detection
-   Supports **USB** and **CSI** cameras
-   Lightweight and optimized for Jetson
-   Simple Python API

------------------------------------------------------------------------

## Summary

-   MediaPipe enables efficient face detection on Jetson
-   Works with both USB and CSI cameras
-   Suitable for real-time vision applications

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
