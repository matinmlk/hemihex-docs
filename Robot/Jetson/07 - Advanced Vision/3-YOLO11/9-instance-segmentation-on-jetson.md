---
sidebar_position: 9
title: Instance Segmentation on Jetson (YOLO)
---

# Instance Segmentation on Jetson

This section demonstrates **instance segmentation** using **Ultralytics
YOLO segmentation models** on NVIDIA Jetson. Examples include **image**,
**video**, and **real-time camera** inference.

------------------------------------------------------------------------

## 1. Enable Optimal Jetson Performance

For best inference speed, enable maximum power and clocks.

### Enable MAX Power Mode

``` bash
sudo nvpmodel -m 2
```

### Enable Jetson Clocks

``` bash
sudo jetson_clocks
```

------------------------------------------------------------------------

## 2. Instance Segmentation on Images

### Enter Demo Directory

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

### Run Image Segmentation Script

``` bash
python3 02.segmentation_image.py
```

Results are saved to:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Image Segmentation)

``` python
from ultralytics import YOLO

model = YOLO("yolo11n-seg.pt")
results = model("assets/zidane.jpg")

for r in results:
    r.show()
    r.save(filename="output/zidane_output.jpg")
```

------------------------------------------------------------------------

## 3. Instance Segmentation on Video

### Run Video Segmentation Script

``` bash
python3 02.segmentation_video.py
```

Output video location:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Video Segmentation)

``` python
import cv2
from ultralytics import YOLO

model = YOLO("yolo11n-seg.pt")
cap = cv2.VideoCapture("videos/people_animals.mp4")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

out = cv2.VideoWriter(
    "output/people_animals_output.mp4",
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

## 4. Real-Time Instance Segmentation

-   **USB Camera**: OpenCV `VideoCapture(0)`
-   **CSI Camera**: GStreamer pipeline (`nvarguscamerasrc`)

Real-time processing follows the same inference logic as video
segmentation.

------------------------------------------------------------------------

## 5. Notes

-   Segmentation models output **pixel-level masks**
-   Suitable for defect contours and object separation
-   Use **Nano segmentation models** for real-time inference
-   Export to **TensorRT** for production deployment

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
