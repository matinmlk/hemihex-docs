---
title: CSI Camera Preview
sidebar_position: 0
---

# CSI camera preview

## 1. Enable CSI camera

If the user has installed the CSI camera but does not have the /dev/video* device, you can enable the CSI camera pins as follows.

### 1.1. Configure pins

Enter the following command in the terminal:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

![image-20250122095925818](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122095925818.png)

Press the arrow keys to select Configure Jetson 24pin CSI Connector , then press Enter to enter the configuration:

![image-20250122095938796](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122095938796.png)

Select Configure for compatible hardware , then press Enter to enter the configuration:

![image-20250122095957948](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122095957948.png)

Select Camera IMX219 Dual , then press Enter to enter the configuration:

![image-20250122100005445](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122100005445.png)

Select Save pin changes , then press Enter to enter the configuration:

![image-20250122100037447](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122100037447.png)

Select Save and reboot to reconfigure pins , and then wait for the system to restart:

![image-20250122100100675](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250122100100675.png)

## 2. Check the video device

```bash
ls /dev/video*
```

The result of the picture is the result of connecting two CSI cameras: generally one CSI camera displays one video device

![image-20250110120003496](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250110120003496.png)

## 3. Preview the camera screen

Enter the following command in the terminal, and the system will automatically pop up the camera screen window: Open the /dev/video0 device by default

```bash
nvgstcapture-1.0
```


![image-20250110120335665](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250110120335665.png)

### 3.1. Specify the camera

If there are multiple cameras, you can specify the camera ID:

```bash
nvgstcapture-1.0 --sensor-id=1
```

![image-20250110120513459](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250110120513459.png)

### 3.2. Specify preview resolution

If there is only one CSI camera, change --sensor-id=1 to --sensor-id=0 :

```bash
nvgstcapture-1.0 --sensor-id=1 --cus-prev-res=1280x720
```

![image-20250110120704726](/img/docs/jetson/05-VisionBasicCourse/5-1/image-20250110120704726.png)
