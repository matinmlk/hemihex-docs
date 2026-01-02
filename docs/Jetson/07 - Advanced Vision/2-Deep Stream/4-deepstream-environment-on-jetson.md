---
sidebar_position: 4
title: DeepStream Environment on Jetson
---

# DeepStream Environment

NVIDIA DeepStream is a high‑performance SDK for building AI‑powered
video analytics applications on **Jetson** and **dGPU** platforms.

This document describes how to install and verify the **DeepStream
environment on Jetson**.

------------------------------------------------------------------------

## 1. System Information

Check your system information before installation.

![System
Information](/img/docs/jetson/07-AdvancedVision/7-4/image-20250121150858152.png)

------------------------------------------------------------------------

## 2. Install DeepStream

During Jetson component installation, ensure **DeepStream 7.1** is
selected.

:::tip
If you are using an automated installer or SDK Manager, enable the
**DeepStream** option explicitly.
:::

![Select DeepStream
Option](/img/docs/jetson/07-AdvancedVision/7-4/image-20250116194316209.png)

------------------------------------------------------------------------

## 3. Verify Installation

Install required GStreamer dependency:

``` bash
sudo apt install -y libgstrtspserver-1.0-0
```

Verify DeepStream version:

``` bash
deepstream-app --version-all
```

If installed correctly, version information will be displayed:

![DeepStream Version
Output](/img/docs/jetson/07-AdvancedVision/7-4/image-20241227113542806.png)

------------------------------------------------------------------------

## 4. Run the Example Application

Navigate to the DeepStream sample configuration directory:

``` bash
cd /opt/nvidia/deepstream/deepstream-7.1/samples/configs/deepstream-app
```

Run the sample pipeline:

``` bash
deepstream-app -c source30_1080p_dec_infer-resnet_tiled_display_int8.txt
```

:::note
This demo may take a long time to start. If the program does not exit
immediately, allow it to continue running.
:::

:::warning
When running high‑load applications, the system may display:

`System throttled due to Over-current.`

This is expected behavior under heavy load.
:::

:::tip
Use a **power adapter rated at least 45W** to avoid throttling issues.
:::

![DeepStream Sample
Output](/img/docs/jetson/07-AdvancedVision/7-4/image-20250102184609430.png)

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson‑based advanced vision and video
analytics workflows.
