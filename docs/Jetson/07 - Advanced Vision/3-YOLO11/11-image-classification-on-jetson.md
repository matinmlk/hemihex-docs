---
sidebar_position: 11
title: Image Classification on Jetson (YOLO)
---

# Image Classification on Jetson

This section demonstrates **image classification** on NVIDIA Jetson
using **Ultralytics YOLO classification models**. Examples include
**image**, **video**, and **real-time camera** classification.

------------------------------------------------------------------------

## 1. Optimize Jetson Performance

Before running classification, ensure Jetson is operating at maximum
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

## 2. Image Classification (Image Input)

### Enter Demo Directory

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

### Run Image Classification Script

``` bash
python3 04.classification_image.py
```

Results are saved to:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Image Classification)

``` python
from ultralytics import YOLO

model = YOLO("yolo11n-cls.pt")
results = model("assets/dog.jpg")

for r in results:
    r.show()
    r.save(filename="output/dog_output.jpg")
```

------------------------------------------------------------------------

## 3. Image Classification (Video Input)

### Run Video Classification Script

``` bash
python3 04.classification_video.py
```

Output video location:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Video Classification)

``` python
import cv2
from ultralytics import YOLO

model = YOLO("yolo11n-cls.pt")
cap = cv2.VideoCapture("videos/cup.mp4")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

out = cv2.VideoWriter(
    "output/cup_output.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    fps,
    (width, height)
)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    annotated = results[0].plot()
    out.write(annotated)

cap.release()
out.release()
```

------------------------------------------------------------------------

## 4. Real-Time Image Classification

-   **USB Camera**: `python3 04.classification_usb_cam.py`
-   **CSI Camera**: `python3 04.classification_csi_cam.py`

------------------------------------------------------------------------

## 5. Notes

-   Classification models output **class probabilities**
-   Suitable for product recognition and defect classification
-   For localization tasks, use YOLO detection models

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
