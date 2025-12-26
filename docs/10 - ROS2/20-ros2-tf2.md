---
sidebar_position: 33
title: ROS 2 TF2 Coordinate Transformation
---

# ROS 2 TF2 Coordinate Transformation

## 1. Introduction to TF2

Coordinate systems are a fundamental concept in robotics. A complete
robotic system often contains many coordinate systems. ROS provides a
powerful tool to manage their relationships: **TF2**.

Reference: [tf: The transform library \|
IEEE](https://ieeexplore.ieee.org/abstract/document/6556373)

------------------------------------------------------------------------

## 2. Coordinate Systems in Robotics

In mobile robotics: - **base_link** represents the robot center -
**laser_link** represents the radar position - **odom** represents
odometry - **map** represents the global reference

These coordinate relationships can be fixed or dynamic. A reliable
coordinate management system is essential.

![Coordinate Example](/img/docs/jetson/10-ROS2/10-23/image-20220528142112163.png)

Transformation consists of **translation + rotation**, represented using
a **4×4 matrix**. TF2 encapsulates all these calculations.

------------------------------------------------------------------------

## 3. TF Command Line Operations

This section demonstrates turtle-following using TF2.

::: note
This demo is recommended to run inside a virtual machine.
:::

------------------------------------------------------------------------

### 3.1 Installing Required Packages

``` bash
sudo apt install ros-${ROS_DISTRO}-turtle-tf2-py ros-humble-tf2-tools
sudo pip3 install transforms3d
sudo apt install ros-${ROS_DISTRO}-rqt-tf-tree
```

![Installation](/img/docs/jetson/10-ROS2/10-23/image-20250905190746493.png)

------------------------------------------------------------------------

### 3.2 Starting the Demo

``` bash
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
ros2 run turtlesim turtle_teleop_key
```

![Turtle Follow](/img/docs/jetson/10-ROS2/10-23/image-20231031174611320.png)

------------------------------------------------------------------------

### 3.3 Viewing the TF Tree

``` bash
ros2 run rqt_tf_tree rqt_tf_tree
```

![TF Tree](/img/docs/jetson/10-ROS2/10-23/image-20231031174844745.png)

------------------------------------------------------------------------

### 3.4 Querying Transform Information

``` bash
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

![TF Output](/img/docs/jetson/10-ROS2/10-23/image-20231031174933242.png)

------------------------------------------------------------------------

### 3.5 TF Visualization in RViz

``` bash
rviz2
```

Set reference frame to `world` and add **TF** plugin.

![RViz TF](/img/docs/jetson/10-ROS2/10-23/image-20231031175440027.png)

------------------------------------------------------------------------

## 4. Static Coordinate Transformation

A static transform defines fixed relations like sensor to base.

### 4.1 Publishing Static Transform

``` bash
ros2 run tf2_ros static_transform_publisher 0 0 3 0 0 3.14 A B
```

![Static TF](/img/docs/jetson/10-ROS2/10-23/image-20250905190855196.png)

------------------------------------------------------------------------

### 4.2 Monitoring Static TF

``` bash
ros2 run tf2_ros tf2_echo A B
```

------------------------------------------------------------------------

### 4.3 Visualizing Static TF

``` bash
rviz2
```

![Static TF RViz](/img/docs/jetson/10-ROS2/10-23/image-20231031180746331.png)

------------------------------------------------------------------------

## 5. Case Introduction

You will: - Implement turtle-follow logic - Implement dynamic TF
broadcaster - Implement coordinate monitoring - Apply PID for motion
conversion

Advanced: - Time-based TF transformation

------------------------------------------------------------------------

## 6. Turtle Follow Principle

![Principle Diagram](/img/docs/jetson/10-ROS2/10-23/image-20220528143750881.png)

Vectors represent direction and distance between turtles. TF
continuously calculates transformations. Velocity is derived from these
vectors.

------------------------------------------------------------------------

## 7. Create TF Package

``` bash
ros2 pkg create pkg_tf --build-type ament_python --dependencies rclpy --node-name turtle_tf_broadcaster
```

This generates a TF broadcaster node template.

------------------------------------------------------------------------

## Maintained by HemiHex

This documentation is maintained by **HemiHex** for ROS 2 TF2 workflows.
