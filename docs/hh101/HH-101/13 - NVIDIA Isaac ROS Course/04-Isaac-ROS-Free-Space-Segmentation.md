---
title: Isaac ROS Free space segmentation
sidebar_position: 4
---

# Isaac ROS Free space segmentation

Isaac ROS Free Space Segmentation official website link：[https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_freespace_segmentation/index.html](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_freespace_segmentation/index.html)

## Overview

Isaac ROS Freespace Segmentation contains an ROS 2 package to produce occupancy grids for navigation. By processing a freespace segmentation mask with the pose of the robot relative to the ground, Bi3D Freespace produces an occupancy grid for Nav2, which is used to avoid obstacles during navigation. This package is GPU accelerated to provide real-time, low latency results in a robotics application. Bi3D Freespace provides an additional occupancy grid source for mobile robots (ground based).

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-4/1.png)

## Quick Experience

To simplify development, we primarily use the Isaac ROS Dev Docker image and demonstrate the effects there. This demonstration does not require any camera device installation; the data stream from the camera is simulated by playing a rosbag file.

**Note: If you wish to install on your own device or connect a camera to develop other features, please refer to the Isaac ROS official website and connect to a designated NVIDIA camera model for custom development.**

Open a terminal and enter the working directory.

```
cd ${ISAAC_ROS_WS}/src
```

Enter the Isaac ROS Dev Docker container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following command to start the system:

```
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
launch_fragments:=bi3d,bi3d_freespace \
interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_bi3d_freespace/rosbag_quickstart_interface_specs.json \
featnet_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/bi3d_proximity_segmentation/featnet.plan \
segnet_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/bi3d_proximity_segmentation/segnet.plan \
max_disparity_values:=10
```

Open a second terminal and enter the container.

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following command:

```
ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_bi3d_freespace/quickstart.bag
```

**View the results**

Open a third terminal and enter the container.

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

Run the following command to view the results.

```
rviz2
```

![image](/img/docs/jetson/13-NVIDIAIsaacROSCourse/13-4/2.png)
