---
sidebar_position: 28
title: ROS 2 Launch Startup File Configuration
---

# 19. ROS 2 Launch Startup File Configuration

## 1. Introduction to Launch

Until now, every time we launched a ROS node, we had to open a new
terminal and run a command. With so many nodes in a robotic system,
doing this every time is cumbersome.

ROS 2 provides **launch files**, which allow you to:

-   Start multiple nodes at once\
-   Configure parameters\
-   Manage system startup more efficiently

ROS 2 launch files can be written in:

-   XML\
-   YAML\
-   Python

This tutorial uses **Python launch files**, which provide the most
flexibility.

### Why Use Python Launch Files?

-   Access to full Python standard libraries\
-   Direct access to ROS 2 launch APIs\
-   More powerful logic and conditional execution

------------------------------------------------------------------------

## 2. Writing a Single Node Launch Program

### 2.1 Create a Package

``` bash
ros2 pkg create learn_launch --build-type ament_python
```

------------------------------------------------------------------------

### 2.2 Create the Launch File

Create a `launch` folder inside the package and add:

``` text
single_node_launch.py
```

Paste the following code:

``` python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='pkg_helloworld_py',
        executable='helloworld',
        output='screen'
    )
    return LaunchDescription([node])
```

------------------------------------------------------------------------

### 2.3 Configure `setup.py`

Add the launch file path so it gets installed:

``` python
import os
from glob import glob

(
    os.path.join('share', package_name, 'launch'),
    glob(os.path.join('launch', '*launch.py'))
)
```

------------------------------------------------------------------------

### 2.4 Compile the Package

``` bash
colcon build --packages-select learn_launch
```

------------------------------------------------------------------------

### 2.5 Run the Launch File

``` bash
ros2 launch learn_launch single_node_launch.py
```

------------------------------------------------------------------------

### 2.6 Source Code Analysis

#### Import Libraries

``` python
from launch import LaunchDescription
from launch_ros.actions import Node
```

#### Define Launch Logic

``` python
def generate_launch_description():
    node = Node(
        package='pkg_helloworld_py',
        executable='helloworld'
    )
    return LaunchDescription([node])
```

**Key Parameters:**

-   `package`: ROS 2 package name\
-   `executable`: Node executable name

------------------------------------------------------------------------

## 3. Writing a Launch Program for Multiple Nodes

### 3.1 Create the Multi-Node Launch File

Create:

``` text
multi_node_launch.py
```

Add the following content:

``` python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    publisher_node = Node(
        package='pkg_topic',
        executable='publisher_demo',
        output='screen'
    )

    subscriber_node = Node(
        package='pkg_topic',
        executable='subscriber_demo',
        output='screen'
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])
```

------------------------------------------------------------------------

### 3.2 Compile the Package

``` bash
colcon build --packages-select learn_launch
```

------------------------------------------------------------------------

### 3.3 Run the Multi-Node Launch File

``` bash
ros2 launch learn_launch multi_node_launch.py
```

------------------------------------------------------------------------

This documentation is maintained by **HemiHex** for ROS 2 launch system
configuration.
