---
sidebar_position: 14
title: Dataset Annotation on Jetson
---

# Dataset Annotation on Jetson

To train custom vision models or improve recognition accuracy, **dataset
collection and annotation** are critical steps. This guide explains how
to collect image data and annotate it using **Label Studio** on NVIDIA
Jetson.

------------------------------------------------------------------------

## 1. Dataset Collection

A common approach is to record a video and extract frames at regular
intervals.

### Extract Images from Video

The following example extracts one image every 15 frames:

``` python
import cv2
import os

video_file = 'orange.mkv'
output_folder = 'orange_images'

os.makedirs(output_folder, exist_ok=True)

cap = cv2.VideoCapture(video_file)
frame_interval = 15
frame_number = 0
saved_frame = 1

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if frame_number % frame_interval == 0:
        cv2.imwrite(f"{output_folder}/{saved_frame}.png", frame)
        saved_frame += 1

    frame_number += 1

cap.release()
print("Frame extraction completed.")
```

------------------------------------------------------------------------

## 2. Install Label Studio

Label Studio is an open-source data annotation platform.

### Install via Docker

``` bash
docker pull heartexlabs/label-studio:latest
```

------------------------------------------------------------------------

## 3. Launch Label Studio

### Set Dataset Permissions

``` bash
sudo chmod 777 ~/ultralytics/ultralytics/data
```

### Run Label Studio Container

``` bash
sudo docker run -it   -p 8080:8080   -v ~/ultralytics/ultralytics/data:/label-studio/data   heartexlabs/label-studio:latest   label-studio
```

### Access Label Studio

-   Local:

        http://localhost:8080

-   Network:

        http://[JETSON_IP]:8080

------------------------------------------------------------------------

## 4. Annotation Workflow

### Create Project

-   Select **Image Annotation**
-   Name the project clearly

### Import Images

Upload images from your dataset directory.

------------------------------------------------------------------------

## 5. Label Configuration Example

``` xml
<View>
  <Image name="image" value="$image"/>
  <RectangleLabels name="label" toName="image">
    <Label value="object"/>
  </RectangleLabels>
</View>
```

------------------------------------------------------------------------

## 6. Export Dataset

Recommended formats: - YOLO - COCO - Pascal VOC

### YOLO Dataset Structure

``` text
dataset/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```

### Example data.yaml

``` yaml
path: dataset
train: images/train
val: images/val

names:
  0: object
```

------------------------------------------------------------------------

## Summary

-   Extract images from real scenes
-   Annotate with Label Studio
-   Export to YOLO format for training
-   Use consistent naming and structure

------------------------------------------------------------------------

Maintained by **HemiHex** for Jetson-based advanced vision workflows.
