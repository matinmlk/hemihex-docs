---
title: USB Camera Preview
sidebar_position: 0
---

# USB camera preview

## 1. View video device

```bash
ls /dev/video*
```

The result of the picture is the result of connecting two CSI cameras and one USB camera: generally, a CSI camera displays one video device, and a USB camera displays two video devices. The USB camera selects the newly added and smaller /dev/video2 call (connecting the USB camera system adds /dev/video2 and /dev/video3 device numbers)

![image-20250110120951302](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110120951302.png)

## 2. GUVCView

GUVCView is an open source software for Linux systems, used to capture and record videos and images, mainly for Webcam cameras.

### 2.1, GUVCView installation

```bash
sudo apt update
sudo apt install guvcview -y
```

![image-20250110121636596](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110121636596.png)

### 2.2, GUVCView use

Enter the application menu bar and click the guvcview icon or enter the startup command in the terminal: Select USB camera, CSI camera has no preview screen

```bash
guvcview
```

![image-20250110121901605](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110121901605.png)

![image-20250110122007230](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110122007230.png)

## 3. VLC

VLC media player is a free and open source multimedia player that supports multiple audio and video formats as well as DVD, audio CD, VCD and various streaming protocols.

### 3.1. VLC installation

```bash
sudo apt update
sudo apt install vlc -y
```

![image-20250110122158140](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110122158140.png)

### 3.2. VLC usage

Enter the application menu bar and click VLC media player icon or enter the start command in the terminal: Select USB camera, CSI camera has no preview screen

```bash
vlc
```

![image-20250110122350426](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110122350426.png)

![image-20250110122452349](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110122452349.png)

Select the device number corresponding to the USB camera:

![image-20250110122526067](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110122526067.png)

![image-20250110123134360](/img/docs/jetson/05-VisionBasicCourse/5-2/image-20250110123134360.png)
