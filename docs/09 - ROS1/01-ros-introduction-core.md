---
sidebar_position: 1
title: ROS Introduction
---

# 1. ROS Introduction

-   **ROS Wiki:** http://wiki.ros.org/\
-   **ROS Tutorials:** http://wiki.ros.org/ROS/Tutorials\
-   **ROS Installation (Ubuntu):**
    http://wiki.ros.org/melodic/Installation/Ubuntu

ROS (Robot Operating System) is an open-source operating system designed
for robots. It provides essential services including hardware
abstraction, low-level device control, commonly used functionality,
inter-process communication, and package management.

The primary goal of ROS is to support code reuse in robot research and
development. ROS follows a distributed architecture based on **nodes**,
which are organized into packages and can be shared and reused across
projects.

------------------------------------------------------------------------

## 1.1 Main Characteristics of ROS

1.  **Distributed Architecture**
2.  **Multi-language Support**
3.  **Good Scalability**
4.  **Open Source (BSD License)**

------------------------------------------------------------------------

## 1.2 Overall Architecture of ROS

-   Open-source Community Level\
-   File System Level\
-   Computation Graph Level

------------------------------------------------------------------------

## 1.2.1 Computation Graph Level

### Node

``` bash
roscore
rosrun turtlesim turtlesim_node
```

``` bash
rosservice call /spawn "x: 3.0
y: 3.0
theta: 90.0
name: 'my_turtle'"
```

``` bash
rqt_graph
```

![ROS Node Graph](/img/docs/jetson/9-ROS1/9-1/001.png)

### `rosnode` Commands

  Command           Description
  ----------------- -------------------
  rosnode list      List nodes
  rosnode info      Node info
  rosnode kill      Kill node
  rosnode ping      Check node
  rosnode machine   List by machine
  rosnode cleanup   Clear stale nodes

------------------------------------------------------------------------

### Message

### `rosmsg` Commands

  Command           Description
  ----------------- ------------------------
  rosmsg show       Display fields
  rosmsg list       List messages
  rosmsg package    Messages in pkg
  rosmsg packages   Packages using message
  rosmsg md5        MD5 checksum

------------------------------------------------------------------------

## 1.2.2 File System Level

![ROS File System](/img/docs/jetson/9-ROS1/9-1/2023042700001.png)

### `rospack` Commands

  Command           Description
  ----------------- ----------------
  rospack help      Help
  rospack list      List packages
  rospack depends   Dependencies
  rospack find      Locate package
  rospack profile   Refresh

------------------------------------------------------------------------

## 1.2.3 Open Source Community Level

-   ROS Distributions
-   Software Libraries
-   ROS Wiki
-   Bug Tracking
-   Mailing Lists
-   ROS Answers
---
sidebar_position: 2
title: ROS Communication & Tools
---

## 1.3 Communication Mechanisms

### Topic (Publish / Subscribe)

![Topic Diagram](/img/docs/jetson/9-ROS1/9-1/012.png)

### `rostopic` Commands

  Command         Description
  --------------- ---------------
  rostopic bw     Bandwidth
  rostopic echo   View messages
  rostopic hz     Frequency
  rostopic list   List topics

------------------------------------------------------------------------

### Service (Client / Server)

![Service Diagram](/img/docs/jetson/9-ROS1/9-1/013.png)

### `rosservice` Commands

  Command           Description
  ----------------- ---------------
  rosservice call   Call service
  rosservice list   List services
  rosservice info   Service info

------------------------------------------------------------------------

### Action

![Action Diagram](/img/docs/jetson/9-ROS1/9-1/010.png)

Supports: - Goal tracking - Continuous feedback - Task cancellation

------------------------------------------------------------------------

## 1.4 Common ROS Components

-   Launch
-   TF
-   RViz
-   Gazebo
-   Navigation
-   MoveIt

------------------------------------------------------------------------

## 1.5 ROS Versions

![ROS Distributions](/img/docs/jetson/9-ROS1/9-1/011.png)

  Version   OS
  --------- --------------
  Noetic    Ubuntu 20.04
  Melodic   Ubuntu 18.04
