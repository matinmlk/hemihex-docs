---
sidebar_position: 21
title: MediaPipe Environment Setup on Jetson
---

# MediaPipe Environment

MediaPipe is a cross-platform framework for building multi-modal
processing pipelines such as image, video, and sensor data processing.
This section explains how to install and verify MediaPipe on NVIDIA
Jetson devices.

------------------------------------------------------------------------

## 1. Install MediaPipe

### Install MediaPipe via pip

``` bash
sudo pip3 install mediapipe
```

------------------------------------------------------------------------

## 2. Install Required Dependencies

### Install OpenCV

``` bash
sudo pip3 install opencv-python
```

### Install dlib (Optional)

``` bash
sudo pip3 install dlib
```

------------------------------------------------------------------------

## 3. Verify Installation

Run the following command to verify MediaPipe installation:

``` bash
python3 -c "import mediapipe as mp; print(mp.__version__)"
```

If successful, the MediaPipe version number will be printed.

------------------------------------------------------------------------

## Summary

-   MediaPipe can be installed directly via pip
-   OpenCV is required for image and video processing
-   dlib is optional depending on use case
-   Always verify installation before using MediaPipe

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
