---
title: Isaac ROS 3D scene reconstruction and mapping
sidebar_position: 7
---

# Isaac ROS 3D scene reconstruction and mapping

Isaac ROS 3D scene reconstruction and mapping official website link：[https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)

## Overview

Isaac ROS Nvblox contains ROS 2 packages for 3D reconstruction and cost maps for navigation.`isaac_ros_nvblox`processes depth and pose to reconstruct a 3D scene in real-time and outputs a 2D costmap for Nav2. The costmap is used in planning during navigation as a vision-based solution to avoid obstacles.

`isaac_ros_nvblox`is designed to work with depth-cameras and/or 3D LiDAR. The package uses GPU acceleration to compute a 3D reconstruction and 2D costmaps using nvblox, the underlying framework-independent C++ library.

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-7/1.png)

## Quick Experience

To simplify development, we primarily use the Isaac ROS Dev Docker image and demonstrate the effects there. This demonstration does not require any camera device installation; it simulates the data stream from a camera by playing a rosbag file.

**Note: If you wish to install on your own device or connect a camera to develop other features, please refer to the Isaac ROS official website and connect to an NVIDIA-specified camera model for your own development.**

Open a terminal and enter the working directory.

```
cd ${ISAAC_ROS_WS}/src
```

Enter the Isaac ROS Dev Docker container.

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following startup command.

```
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py \
rosbag:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_nvblox/quickstart \
navigation:=False
```

Run Results

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-7/2.png)
