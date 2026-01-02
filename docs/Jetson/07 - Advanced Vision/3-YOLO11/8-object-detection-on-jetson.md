---
sidebar_position: 8
title: Object Detection on Jetson (YOLO)
---

# Object Detection on Jetson (Ultralytics YOLO)

This section demonstrates **object detection** on NVIDIA Jetson using
**Ultralytics YOLO**. It covers detection on **images**, **videos**, and
**real-time camera streams** with Jetson performance optimization.

------------------------------------------------------------------------

## 1. Enable Optimal Jetson Performance

Before running inference, configure the Jetson board for maximum
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

## 2. Object Detection on Images

### Enter Demo Directory

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

### Run Image Detection Script

``` bash
python3 01.detection_image.py
```

Detection results are saved to:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Image Detection)

``` python
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
results = model("assets/bus.jpg")

for r in results:
    r.show()
    r.save(filename="output/bus_output.jpg")
```

------------------------------------------------------------------------

## 3. Object Detection on Videos

### Run Video Detection Script

``` bash
python3 01.detection_video.py
```

Output video location:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Video Detection)

``` python
import cv2
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
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

## 4. Real-Time Object Detection

### USB Camera

``` bash
python3 02.detection_usb_camera.py
```

### CSI Camera

``` bash
python3 03.detection_csi_camera.py
```

------------------------------------------------------------------------

## 5. Best Practices

-   Use **Nano models (yolo11n)** for real-time inference
-   Prefer **CSI cameras** for lower latency
-   Always enable MAX power mode
-   Export models to **TensorRT** for production

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
