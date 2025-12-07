---
sidebar_position: 0
title: ROS Introduction
---

# 1. ROS Introduction

-   **ROS Wiki:** http://wiki.ros.org/\
-   **ROS Tutorials:** http://wiki.ros.org/ROS/Tutorials\
-   **ROS Installation (Melodic on Ubuntu):**
    http://wiki.ros.org/melodic/Installation/Ubuntu

ROS (Robot Operating System) is an open-source operating system
framework designed for robotics. It provides essential services such as
hardware abstraction, low-level device control, implementation of common
functions, inter-process communication, and package management. It also
offers the tools and libraries required to acquire, compile, write, and
run code across multiple computers.

The primary goal of ROS is to support code reuse for robot research and
development. ROS follows a distributed process model (called **nodes**),
which are organized into packages that are easy to share and publish.
ROS also supports collaborative development through shared repositories.

## 1.1 Main Characteristics of ROS

1.  Distributed Architecture\
2.  Multi-language Support\
3.  High Scalability\
4.  Open Source BSD License

## 1.2 Overall Architecture of ROS

-   Open Source Community Level\
-   File System Level\
-   Computational Graph Level

### 1.2.1 Computational Graph Level

``` bash
roscore
rosrun turtlesim turtlesim_node
```

``` bash
rqt_graph
```

## 1.3 Communication Mechanisms

### 1.3.1 Topic

### 1.3.2 Service

### 1.3.3 Action

## 1.4 Common Components

-   Launch\
-   TF\
-   Rviz\
-   Gazebo\
-   Navigation\
-   MoveIt

## 1.5 Release Versions

Reference: http://wiki.ros.org/Distributions
