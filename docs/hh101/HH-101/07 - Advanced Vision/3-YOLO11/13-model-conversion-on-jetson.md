---
sidebar_position: 13
title: Model Conversion on Jetson (YOLO to TensorRT)
---

# Model Conversion on Jetson

This section explains how to convert **YOLO models** on NVIDIA Jetson
devices for optimal inference performance using **TensorRT**.

The standard conversion pipeline is:

    PyTorch (.pt) → ONNX (.onnx) → TensorRT (.engine)

------------------------------------------------------------------------

## 1. Enable Optimal Jetson Performance

Before model conversion or inference, configure Jetson for maximum
performance.

### Enable MAX Power Mode

``` bash
sudo nvpmodel -m 2
```

### Enable Jetson Clocks

``` bash
sudo jetson_clocks
```

------------------------------------------------------------------------

## 2. Model Conversion Overview

TensorRT provides the fastest inference on Jetson. Ultralytics YOLO
supports **direct export** to TensorRT, automatically generating an
intermediate ONNX model.

------------------------------------------------------------------------

## 3. CLI Model Conversion

Navigate to the Ultralytics directory:

``` bash
cd ~/ultralytics/ultralytics
```

Export YOLO models to TensorRT:

``` bash
yolo export model=yolo11n.pt format=engine
# yolo export model=yolo11n-seg.pt format=engine
# yolo export model=yolo11n-pose.pt format=engine
# yolo export model=yolo11n-cls.pt format=engine
# yolo export model=yolo11n-obb.pt format=engine
```

The generated `.engine` file will be saved alongside the original model.

------------------------------------------------------------------------

## 4. Python Model Conversion

Navigate to the demo directory:

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

Run the conversion script:

``` bash
python3 model_pt_onnx_engine.py
```

### Example Python Code

``` python
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
model.export(format="engine")
```

------------------------------------------------------------------------

## 5. Model Inference

### USB Camera Inference (CLI)

``` bash
yolo predict model=yolo11n.engine source=0 show save=False
```

### ONNX Inference

``` bash
yolo predict model=yolo11n.onnx source=0 show save=False
```

:::note
CLI inference supports USB cameras. For CSI cameras, use Python-based
inference.
:::

------------------------------------------------------------------------

## 6. Common Issues

### onnxslim Error

If you encounter an `onnxslim` error:

``` bash
sudo pip3 install onnxslim
```

Then re-run the export command.

------------------------------------------------------------------------

## 7. Summary

-   Always enable **MAX power mode** and **Jetson clocks**
-   TensorRT delivers the best inference performance
-   Ultralytics simplifies conversion using CLI and Python APIs
-   Use `.engine` models for production deployment

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
