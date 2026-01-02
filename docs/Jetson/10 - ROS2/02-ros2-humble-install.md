---
sidebar_position: 12
title: Humble Installation
---

# 2. Installing Humble in ROS 2

-   The ROS 2 **Humble** installation supports **Ubuntu 22.04**.
-   If you need a different ROS 2 version, replace `humble` in all
    commands with the desired version (for example, `foxy`).

------------------------------------------------------------------------

## 1. Set the Locale

First, ensure your system supports **UTF-8** encoding:

``` bash
locale  # Check for UTF-8 support

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify the configuration
```

::: note
The locale can differ, but it **must support UTF-8**.
:::

------------------------------------------------------------------------

## 2. Set Up the Software Source

Enable the Ubuntu universe repository:

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 apt repository and authorize the GPG key:

``` bash
sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to the source list:

``` bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

------------------------------------------------------------------------

## 3. Install Humble

Update the package index:

``` bash
sudo apt update
```

Upgrade existing packages:

``` bash
sudo apt upgrade
```

Install the **ROS 2 desktop version**:

``` bash
sudo apt install ros-humble-desktop python3-argcomplete
```

Install the **colcon build tool**:

``` bash
sudo apt install python3-colcon-common-extensions
```

------------------------------------------------------------------------

## 4. Configure the Environment

Source ROS 2 in every new terminal:

``` bash
source /opt/ros/humble/setup.bash
```

To make this permanent:

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

At this stage, **ROS 2 Humble is fully installed and configured**.

------------------------------------------------------------------------

## 5. Run the Publisher and Subscriber Example Nodes

Run the built-in ROS 2 demo nodes to verify installation.

### Publisher

``` bash
ros2 run demo_nodes_cpp talker
```

### Subscriber

``` bash
ros2 run demo_nodes_cpp listener
```

------------------------------------------------------------------------

## 6. Run and Test the Turtle Simulator

Start the turtle simulator:

``` bash
ros2 run turtlesim turtlesim_node
```

Run the keyboard control node:

``` bash
ros2 run turtlesim turtle_teleop_key
```

You can now control the turtle using your keyboard.
