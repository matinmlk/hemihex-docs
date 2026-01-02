---
sidebar_position: 5
title: Four Types of Traffic Detection (DeepStream)
---

# Four Types of Traffic Detection (DeepStream)

This document demonstrates **four traffic-related detection scenarios**
using NVIDIA DeepStream example applications. These demos showcase
detection and counting of:

-   People
-   Vehicles
-   Traffic signs
-   Bicycles

:::note
These examples are **basic demonstrations**. For production or custom
use cases, further development and configuration are required.
:::

------------------------------------------------------------------------

## 1. Video Detection

This example runs detection on a **video file**.

### Step 1: Enter the Sample Application Directory

``` bash
cd /opt/nvidia/deepstream/deepstream-7.1/sources/apps/sample_apps/deepstream-test1
```

### Step 2: Compile the Source Code

Refer to the `README` file in the same directory.

``` bash
sudo make CUDA_VER=12.6
```

:::tip
On factory images, the application is often already compiled and may not
require recompilation.
:::

### Step 3: Run the Demo

Run the application using the provided configuration file:

``` bash
./deepstream-test1-app dstest1_config.yml
```

Or run directly with a sample video:

``` bash
./deepstream-test1-app ../../../../samples/streams/sample_720p.h264
```

### Example Output

![Video Detection
Result](/img/docs/jetson/07-AdvancedVision/7-5/image-20250102191922399.png)

------------------------------------------------------------------------

### Configuration File: `dstest1_config.yml`

``` yaml
source:
  location: ../../../../samples/streams/sample_720p.h264

streammux:
  batch-size: 1
  batched-push-timeout: 40000
  width: 1920
  height: 1080

primary-gie:
  plugin-type: 0
  config-file-path: dstest1_pgie_config.yml
```

------------------------------------------------------------------------

## 2. Real-Time Detection

Real-time detection uses **live camera input**.

------------------------------------------------------------------------

### 2.1 USB Camera Detection

``` bash
cd /opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app
deepstream-app -c source1_usb_dec_infer_resnet_int8.txt
```

![USB Camera
Detection](/img/docs/jetson/07-AdvancedVision/7-5/image-20250102203840338.png)

------------------------------------------------------------------------

### 2.2 CSI Camera Detection

``` bash
cd /opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app
deepstream-app -c source1_csi_dec_infer_resnet_int8.txt
```

![CSI Camera
Detection](/img/docs/jetson/07-AdvancedVision/7-5/image-20250102202305616.png)

------------------------------------------------------------------------

## Summary

-   DeepStream provides ready-to-run **traffic detection demos**
-   Supports video files, USB cameras, and CSI cameras
-   Configuration files control inference pipelines
-   Suitable as a starting point for intelligent traffic systems

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision and
DeepStream workflows.
