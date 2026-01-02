---
sidebar_position: 7
title: YOLO CLI Use on Jetson
---

# CLI Use (YOLO on Jetson)

This section explains how to use the **YOLO command-line interface
(CLI)** on NVIDIA Jetson devices for model inference and testing.

------------------------------------------------------------------------

## 1. Download Source Code

Clone the YOLO repository and enter the working directory:

``` bash
git clone https://github.com/ultralytics/ultralytics.git
cd ultralytics
```

(Optional) Install dependencies:

``` bash
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

------------------------------------------------------------------------

## 2. Enable Optimal Performance on Jetson

To achieve the best inference performance, configure Jetson for maximum
performance.

### 2.1 Enable MAX Power Mode

``` bash
sudo nvpmodel -m 0
```

Verify the current power mode:

``` bash
sudo nvpmodel -q
```

------------------------------------------------------------------------

### 2.2 Enable Jetson Clocks

Lock CPU, GPU, and memory clocks:

``` bash
sudo jetson_clocks
```

Restore default clocks if needed:

``` bash
sudo jetson_clocks --restore
```

------------------------------------------------------------------------

## 3. YOLO CLI Prediction Examples

### 3.1 Image Prediction

``` bash
yolo predict model=yolov8n.pt source=image.jpg device=0
```

------------------------------------------------------------------------

### 3.2 Video Prediction

``` bash
yolo predict model=yolov8n.pt source=video.mp4 device=0
```

------------------------------------------------------------------------

### 3.3 USB Camera Prediction

``` bash
yolo predict model=yolov8n.pt source=0 device=0
```

::: note
`source=0` corresponds to `/dev/video0`.
:::

------------------------------------------------------------------------

### 3.4 CSI Camera Prediction (GStreamer)

``` bash
yolo predict model=yolov8n.pt source="nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
```

------------------------------------------------------------------------

## 4. Output Results

Prediction results are saved by default to:

``` text
runs/detect/predict/
```

This directory contains: - Annotated images or videos - Detection
metadata

------------------------------------------------------------------------

## 5. Verification

Check YOLO environment status:

``` bash
yolo checks
```

Expected output includes: - CUDA available - GPU detected - Torch
installed correctly

------------------------------------------------------------------------

## Summary

-   YOLO CLI enables rapid testing on Jetson
-   Supports image, video, USB, and CSI camera inputs
-   Use MAX power mode for best performance
-   Suitable for development and validation workflows

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
