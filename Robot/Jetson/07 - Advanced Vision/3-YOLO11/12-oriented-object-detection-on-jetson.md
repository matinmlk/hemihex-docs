---
sidebar_position: 12
title: Oriented Object Detection on Jetson (YOLO OBB)
---

# Oriented Object Detection on Jetson

Oriented Object Detection (OBB) extends traditional object detection by
predicting **rotated bounding boxes**. This is especially useful for
aerial imagery, traffic scenes, manufacturing parts, and objects with
arbitrary orientations.

This section demonstrates **Ultralytics YOLO OBB** on NVIDIA Jetson for
image, video, and real-time camera inference.

------------------------------------------------------------------------

## 1. Enable Optimal Jetson Performance

Before running inference, configure Jetson for maximum performance.

### Enable MAX Power Mode

``` bash
sudo nvpmodel -m 2
```

### Enable Jetson Clocks

``` bash
sudo jetson_clocks
```

------------------------------------------------------------------------

## 2. Oriented Object Detection on Images

### Enter Demo Directory

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

### Run Image OBB Detection

``` bash
python3 05.obb_image.py
```

Results are saved to:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Image OBB)

``` python
from ultralytics import YOLO

model = YOLO("yolo11n-obb.pt")
results = model("assets/car.jpg")

for r in results:
    r.show()
    r.save(filename="output/car_obb_output.jpg")
```

------------------------------------------------------------------------

## 3. Oriented Object Detection on Video

### Run Video OBB Detection

``` bash
python3 05.obb_video.py
```

Output video location:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Video OBB)

``` python
import cv2
from ultralytics import YOLO

model = YOLO("yolo11n-obb.pt")
cap = cv2.VideoCapture("videos/street.mp4")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

out = cv2.VideoWriter(
    "output/street_obb_output.mp4",
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

## 4. Real-Time Oriented Object Detection

-   **USB Camera**: `python3 05.obb_usb_cam.py`
-   **CSI Camera**: `python3 05.obb_csi_cam.py`

Both modes support real-time oriented bounding box visualization.

------------------------------------------------------------------------

## 5. Notes

-   OBB models output rotated bounding boxes
-   Suitable for traffic analysis, aerial views, and manufacturing
    inspection
-   Use Nano OBB models for real-time inference on Jetson
-   Export to TensorRT for production deployment

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
