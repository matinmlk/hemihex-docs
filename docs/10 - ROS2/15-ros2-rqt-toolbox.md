---
sidebar_position: 27
title: ROS 2 RQt Toolbox
---

# 18. ROS 2 RQt Toolbox

-   This tutorial folder contains demo animations that visually
    illustrate the process of implementing the examples in this section.

![RQt Toolbox Overview](/img/docs/jetson/10-ROS2/10-18/image-20250905172110533.png)

------------------------------------------------------------------------

## 1. Introduction to RQt

RQt is a modular visualization and debugging tool provided by ROS. Like
RViz, it is built on the Qt framework. Before using it, you need to
install it and then start it using the `rqt` command.

------------------------------------------------------------------------

## 2. Installation

-   In most cases, the RQt toolbox is installed by default when you
    install the **ROS 2 desktop version**.
-   If you installed a minimal version of ROS and RQt is missing, you
    can install it manually using:

``` bash
sudo apt install ros-${ROS_DISTRO}-rqt*
```

------------------------------------------------------------------------

## 3. Startup

Common methods to start RQt include:

### Method 1

``` bash
rqt
```

![RQt Startup Method 1](/img/docs/jetson/10-ROS2/10-18/image-20250905172216440.png)

------------------------------------------------------------------------

### Method 2

``` bash
ros2 run rqt_gui rqt_gui
```

![RQt Startup Method 2](/img/docs/jetson/10-ROS2/10-18/image-20250905172235292.png)

------------------------------------------------------------------------

## 4. Plugin Usage

After starting RQt, you can load different plugins through the
**Plugins** menu.

![RQt Plugin Menu](/img/docs/jetson/10-ROS2/10-18/3.7.2%20RQT%20Toolbox.gif)

The plugin system includes tools for:

-   Topics\
-   Services\
-   Actions\
-   Parameters\
-   Logging

You can select plugins as needed to simplify debugging and system
introspection. Example use cases are shown below.

------------------------------------------------------------------------

### 4.1 Topic Plugin

Add the **Topic** plugin and send speed commands to control the turtle's
movement.

![RQt Topic Plugin](/img/docs/jetson/10-ROS2/10-18/3.7.2%20RQT%20Toolbox%20topic.gif)

------------------------------------------------------------------------

### 4.2 Service Plugin

Add the **Service** plugin and send a request to spawn a turtle at a
specified location.

![RQt Service
Plugin](/img/docs/jetson/10-ROS2/10-18/3.7.2RQT%20Toolbox%20service-1698722363522-4.gif)

------------------------------------------------------------------------

### 4.3 Parameter Plugin

Use the **Parameter** plugin to dynamically change the background color
of the turtle simulation window.

![RQt Parameter Plugin](/img/docs/jetson/10-ROS2/10-18/3.7.2%20RQT%20Toolbox%20param.gif)

------------------------------------------------------------------------

This documentation is maintained by **HemiHex** for ROS 2 visualization
and debugging workflows.
