---
sidebar_position: 10
title: Pose Estimation on Jetson (YOLO)
---

# Pose Estimation on Jetson

This section demonstrates **human pose estimation** on NVIDIA Jetson
using **Ultralytics YOLO Pose models**. Examples include **image**,
**video**, and **real-time camera** inference.

------------------------------------------------------------------------

## 1. Enable Optimal Jetson Performance

Before running pose estimation, ensure Jetson is running at maximum
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

## 2. Pose Estimation on Images

### Enter Demo Directory

``` bash
cd ~/ultralytics/ultralytics/yahboom_demo
```

### Run Image Pose Estimation

``` bash
python3 03.pose_image.py
```

Results are saved to:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Image Pose Estimation)

``` python
from ultralytics import YOLO

model = YOLO("yolo11n-pose.pt")
results = model("assets/people.jpg")

for r in results:
    r.show()
    r.save(filename="output/people_pose_output.jpg")
```

------------------------------------------------------------------------

## 3. Pose Estimation on Video

### Run Video Pose Estimation

``` bash
python3 03.pose_video.py
```

Output video location:

``` text
~/ultralytics/ultralytics/output/
```

------------------------------------------------------------------------

### Sample Code (Video Pose Estimation)

``` python
import cv2
from ultralytics import YOLO

model = YOLO("yolo11n-pose.pt")
cap = cv2.VideoCapture("videos/people.mp4")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

out = cv2.VideoWriter(
    "output/people_pose_output.mp4",
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

## 4. Real-Time Pose Estimation

-   **USB Camera**: OpenCV `VideoCapture(0)`
-   **CSI Camera**: GStreamer pipeline (`nvarguscamerasrc`)

Real-time processing follows the same inference logic as video pose
estimation.

------------------------------------------------------------------------

## 5. Notes

-   Pose models output **keypoints** for each detected person
-   Suitable for motion tracking and activity analysis
-   Use **Nano pose models** for real-time inference
-   Export models to **TensorRT** for production deployment

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
