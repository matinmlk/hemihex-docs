---
sidebar_position: 22
title:  Distributed Communication - Basics
---

# ROS 2 Distributed Communication (Part 1: Basics)

## 1. Concept

Multi-machine communication, also known as **distributed
communication**, refers to a communication strategy that enables data
exchange between different hosts over a network.

ROS 2 itself is a distributed communication framework. The middleware
underlying ROS 2 is **DDS (Data Distribution Service)**. When running on
the same network, distributed communication is achieved through the
**DDS Domain ID mechanism (`ROS_DOMAIN_ID`)**.

### How It Works

Before starting a node, you can set a domain ID value. Nodes with the
**same domain ID** can freely discover and communicate with each other.
If the domain IDs are **different**, communication is not possible.

By default, all nodes start with:

``` bash
ROS_DOMAIN_ID=0
```

This means that as long as devices are on the **same network**,
distributed communication works automatically without extra
configuration.

------------------------------------------------------------------------

## 2. Typical Applications

Distributed communication is widely used in scenarios such as:

-   Unmanned vehicle platooning\
-   Drone swarms\
-   Remote robot control

All of these rely on distributed data exchange.

------------------------------------------------------------------------

## 3. Implementation

### 3.1 Default Implementation

Distributed communication is achieved simply by placing all devices on
the **same network**, such as:

-   The same WiFi network\
-   The same router

On Windows, setting a virtual machine to **bridge mode** places it on
the same network as the host.

------------------------------------------------------------------------

## 4. Test Setup

Assume two hosts:

-   **Host A**
-   **Host B**

These can be:

-   Virtual machines\
-   Raspberry Pi\
-   Jetson devices\
-   x86/ARM hosts

All that is required is that they run the **same ROS 2 version**.

------------------------------------------------------------------------

### Step 1: Execute on Host A

``` bash
ros2 run demo_nodes_py talker
```

------------------------------------------------------------------------

### Step 2: Execute on Host B

``` bash
ros2 run demo_nodes_py listener
```

If the listener successfully receives messages from the talker, then
**multi-machine communication is working**.

------------------------------------------------------------------------

## 5. Remote Turtle Control Example

### Host A

``` bash
ros2 run turtlesim turtlesim_node
```

### Host B

``` bash
ros2 run turtlesim turtle_teleop_key
```

The turtle on Host A can now be controlled remotely from Host B.
