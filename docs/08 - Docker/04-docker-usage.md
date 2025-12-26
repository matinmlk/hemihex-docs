---
sidebar_position: 4
title: Docker Usage (ROS 1 Environment)
---

# Docker Usage

This tutorial demonstrates how to build a **ROS 1 Melodic** environment
inside a Docker container based on `ubuntu:18.04`.

::: note
All commands are executed **inside the Docker container**. Administrator
privileges (`sudo`) are not required inside Docker.
:::

------------------------------------------------------------------------

## 1. Start the Image

Start the Ubuntu 18.04 image in interactive mode:

``` bash
docker run -it ubuntu:18.04 /bin/bash
```

![Start Container](/img/docs/jetson/8-docker/8-4/image-20250105145841732.png)

------------------------------------------------------------------------

## 2. ROS Environment Construction

### 2.1 Update System Software

Ensure system packages are up to date:

``` bash
apt update && apt upgrade
```

![System Update](/img/docs/jetson/8-docker/8-4/image-20250105150409953.png)

------------------------------------------------------------------------

### 2.2 Determine the Language Environment

ROS requires UTF-8 support.

#### 2.2.1 Verify the System Environment

``` bash
locale
```

![Locale Check](/img/docs/jetson/8-docker/8-4/image-20250105150553931.png)

------------------------------------------------------------------------

#### 2.2.2 Set the UTF-8 Environment

If UTF-8 is not enabled, run:

``` bash
apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
echo "export LANG=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

![UTF-8 Setup](/img/docs/jetson/8-docker/8-4/image-20250105150825784.png)

------------------------------------------------------------------------

### 2.3 Set Software Source

Install required tools:

``` bash
apt install lsb-core
```

Add ROS software source:

``` bash
sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

![ROS Source](/img/docs/jetson/8-docker/8-4/image-20250105151200285.png)

------------------------------------------------------------------------

### 2.4 Set ROS Key

``` bash
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

![ROS Key](/img/docs/jetson/8-docker/8-4/image-20250105151255171.png)

------------------------------------------------------------------------

### 2.5 Install ROS 1 Desktop Version

Install ROS Melodic desktop-full:

``` bash
apt update && apt upgrade
apt install ros-melodic-desktop-full -y
```

![ROS Install](/img/docs/jetson/8-docker/8-4/image-20250105151509965.png)

During installation, select your region and city when prompted.

![Timezone Select](/img/docs/jetson/8-docker/8-4/image-20250105153026285.png)

------------------------------------------------------------------------

### 2.6 Install ROS Dependencies

``` bash
apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```

![Dependencies](/img/docs/jetson/8-docker/8-4/image-20250105153047221.png)

------------------------------------------------------------------------

### 2.7 rosdep Initialization

``` bash
rosdep init
rosdep update
```

::: warning
If `rosdep init` fails due to GitHub access issues, resolve DNS manually
by mapping `raw.githubusercontent.com` in `/etc/hosts`.
:::

Install editor if needed:

``` bash
apt install nano -y
```

------------------------------------------------------------------------

### 2.8 Setting Environment Variables

#### 2.8.1 Temporary Settings

``` bash
source /opt/ros/melodic/setup.bash
```

#### 2.8.2 Automatic Settings (Recommended)

``` bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

------------------------------------------------------------------------

## 3. Submit the Image

After completing configuration, exit the container:

``` bash
exit
```

Find the container ID:

``` bash
docker ps -a
```

Commit the container as a new image:

``` bash
docker commit <CONTAINER_ID> ros-melodic:18.04
```

------------------------------------------------------------------------

## 4. Verify the Image / ROS

### 4.1 Start the Image

``` bash
docker run -it ros-melodic:18.04 /bin/bash
```

------------------------------------------------------------------------

### 4.2 Verify the ROS Environment

``` bash
roscore
```

If ROS starts successfully, the environment is ready.

------------------------------------------------------------------------

Maintained by **HemiHex**.
