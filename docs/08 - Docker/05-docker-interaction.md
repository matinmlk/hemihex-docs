---
sidebar_position: 5
title: Docker Interaction
---

# Docker Interaction

This tutorial introduces **data and hardware interaction between the
host system and Docker containers**, including scripting, shared data,
networking, and hardware access.

------------------------------------------------------------------------

## 1. Bash Script

**Bash (Bourne Again Shell)** is a scripting language used to automate
tasks. It allows multiple Docker commands to be combined and executed
sequentially, improving readability and reusability.

::: note
Using scripts helps reduce long command lines with many parameters and
makes container startup easier to understand.
:::

------------------------------------------------------------------------

### 1.1 Basic Docker Script

**Basic template:**

``` bash
#!/bin/bash

docker run -it <image_name>:<tag> /bin/bash
```

------------------------------------------------------------------------

**Example script** (`ros_melodic.sh`):

``` bash
#!/bin/bash

docker run -it ros_melodic:1.0 /bin/bash
```

Run the script:

``` bash
sh ros_melodic.sh
```

------------------------------------------------------------------------

### 1.2 Run the Script

``` bash
sh ros_melodic.sh
```

------------------------------------------------------------------------

## 2. Shared Data

Docker supports sharing host resources with containers.

------------------------------------------------------------------------

### 2.1 Shared Folder

Mount a host directory into the container using `-v`.

**Example:**

``` bash
#!/bin/bash

docker run -it -v /home/hemihex/share:/share ros_melodic:1.0 /bin/bash
```

This maps the host folder `/home/hemihex/share` to `/share` inside the
container.

------------------------------------------------------------------------

### 2.2 Shared Network

Share the host network with Docker using `--net=host`.

``` bash
#!/bin/bash

docker run -it --net=host -v /home/hemihex/share:/share ros_melodic:1.0 /bin/bash
```

If `ifconfig` is not available inside the container, install networking
tools:

``` bash
sudo apt install net-tools -y
ifconfig
```

------------------------------------------------------------------------

## 3. Shared Hardware

Docker containers can access host hardware devices when explicitly
enabled.

------------------------------------------------------------------------

### 3.1 Shared Graphical Interface (GUI)

To run GUI applications inside Docker and display them on the host:

``` bash
#!/bin/bash

xhost +

docker run -it --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/hemihex/share:/share ros_melodic:1.0 /bin/bash
```

::: warning
Using `xhost +` disables access control for X11. Use it only in trusted
environments.
:::

------------------------------------------------------------------------

### 3.2 Ordinary Camera

USB cameras can be mapped into Docker containers using `/dev/video*`.

If only one camera is connected, it is typically:

``` text
/dev/video0
```

**Example:**

``` bash
#!/bin/bash

xhost +

docker run -it --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/hemihex/share:/share --device=/dev/video0 ros_melodic:1.0 /bin/bash
```

------------------------------------------------------------------------

### 3.3 Depth Camera

Depth cameras (e.g., RGB‑D cameras) often expose multiple `/dev/video*`
devices.

To identify them:

``` bash
ls /dev/video*
```

Map all required devices into the container:

``` bash
--device=/dev/video0 --device=/dev/video1 --device=/dev/video2
```

------------------------------------------------------------------------

## Summary

-   Bash scripts simplify Docker startup commands
-   Volumes (`-v`) enable host--container file sharing
-   `--net=host` allows shared networking
-   GUI applications require X11 forwarding
-   Hardware devices must be explicitly mapped

------------------------------------------------------------------------

Maintained by **HemiHex**.
