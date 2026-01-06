---
title: Isaac ROS DNN stereoscopic depth
sidebar_position: 3
---

# Isaac ROS DNN stereoscopic depth

Isaac ROS DNN Stereo Depth official website link：[https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html)

## Overview

The vision depth perception problem is generally useful in many fields of robotics such as estimating the pose of a robotic arm in an object manipulation task, estimating distance of static or moving targets in autonomous robot navigation, tracking targets in delivery robots and so on. Isaac ROS DNN Stereo Depth is targeted at two Isaac applications, Isaac Manipulator and Isaac Perceptor. In Isaac Manipulator application, ESS is deployed in Isaac ROS cuMotion package as a plug-in node to provide depth perception maps for robot arm motion planning and control. In this scenario, multi-camera stereo streams of industrial robot arms on a table task are passed to ESS to obtain corresponding depth streams. The depth streams are used to segment the relative distance of robot arms from corresponding objects on the table; thus providing signals for collision avoidance and fine-grain control. Similarly, the Isaac Perceptor application uses several Isaac ROS packages, namely, Isaac ROS Nova, Isaac ROS Visual Slam, Isaac ROS Stereo Depth (ESS), Isaac ROS Nvblox and Isaac ROS Image Pipeline.

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-3/1.png)

## Quick Experience

To simplify development, we primarily use the Isaac ROS Dev Docker image and demonstrate the effects there. This demonstration does not require any camera device installation; it simulates the camera data stream by playing a rosbag file.

**Note: If you wish to install on your own device or connect a camera to develop other features, please refer to the Isaac ROS official website and connect to a designated NVIDIA camera model for custom development.**

Open a terminal and enter the working directory

```
cd ${ISAAC_ROS_WS}/src
```

Enter the Isaac ROS Dev Docker container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following startup command, where threshold:=0.0 can be changed to 0.4 at startup, which will have different effects.

```
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=ess_disparity \
engine_file_path:=${ISAAC_ROS_WS:?}/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine \
threshold:=0.0
```

Open a second terminal and enter the container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following command

```
ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_ess/rosbags/ess_rosbag \
--remap /left/camera_info:=/left/camera_info_rect /right/camera_info:=/right/camera_info_rect
```

**View the results**

Open a third terminal and enter the container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following command,

```
ros2 run isaac_ros_ess isaac_ros_ess_visualizer.py
```

When threshold is set to 0.0, the display results are as follows:

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-3/2.png)

When the threshold is set to 0.4, the results are as follows:

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-3/3.png)
