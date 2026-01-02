---
sidebar_position: 6
title: YOLO Environment Construction
---

# YOLO Environment Construction

This document describes how to **set up a YOLO development and inference
environment** on NVIDIA Jetson platforms using Ultralytics YOLO with GPU
acceleration.

------------------------------------------------------------------------

## 1. System Information

Verify your system environment before installation.

![System
Information](/img/docs/jetson/07-AdvancedVision/7-6/image-20250121150858152.png)

------------------------------------------------------------------------

## 2. Preliminary Preparation

Update system packages and ensure `pip` is available:

``` bash
sudo apt update
sudo apt install python3-pip -y
sudo pip install -U pip
```

------------------------------------------------------------------------

## 3. Install Ultralytics YOLO

Install the Ultralytics YOLO framework with export support:

``` bash
sudo pip3 install ultralytics[export]
```

Reboot the system after installation:

``` bash
sudo reboot
```

------------------------------------------------------------------------

## 4. Configure GPU Acceleration

::: note
Torch and Torchvision were installed in previous sections. The following
steps install additional GPU-related dependencies.
:::

### 4.1 Torch

``` bash
sudo pip3 install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
```

### 4.2 Torchvision

``` bash
sudo pip3 install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl
```

### 4.3 cuSPARSELt

``` bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev
```

### 4.4 ONNX Runtime GPU

``` bash
sudo pip3 install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
```

::: warning
`onnxruntime-gpu` requires a specific NumPy version.
:::

Install the compatible NumPy version:

``` bash
sudo pip3 install numpy==1.23.5
```

------------------------------------------------------------------------

## 5. Verify the Installation

### Verify Ultralytics

``` bash
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

### Verify Torch

``` bash
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

### Verify Torchvision

``` bash
python3 -c "import torchvision; print(torchvision.__version__)"
```

### Verify NumPy

``` bash
python3 -c "import numpy; print(numpy.__version__)"
```

![Verification
Output](/img/docs/jetson/07-AdvancedVision/7-6/image-20241230112550391.png)

------------------------------------------------------------------------

## 6. Common Errors

### 6.1 Cannot Uninstall `sympy`

**Error:** Unable to uninstall `sympy`

![Sympy
Error](/img/docs/jetson/07-AdvancedVision/7-6/image-20241230111303169.png)

**Solution:**

``` bash
sudo apt remove python3-sympy -y
```

Reinstall PyTorch afterward if required.

------------------------------------------------------------------------

### 6.2 CSI Camera Cannot Be Called

If the CSI camera does not work with YOLO inference:

-   Rebuild OpenCV from source
-   Ensure **CUDA** and **GStreamer** support are enabled
-   Remove old OpenCV versions before reinstalling

This resolves most CSI camera access issues on Jetson platforms.

------------------------------------------------------------------------

## References

-   Ultralytics YOLO Documentation: https://docs.ultralytics.com/
-   NVIDIA Jetson AI Lab: https://pypi.jetson-ai-lab.dev/

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision and YOLO
workflows.
