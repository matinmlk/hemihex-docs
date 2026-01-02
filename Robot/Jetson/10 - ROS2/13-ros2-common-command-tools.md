---
sidebar_position: 25
title:  Common Command Tools
---

# 16. ROS 2 Common Command Tools

## 1. Package Management Tool: `ros2 pkg`

### 1.1 `ros2 pkg create`

**Function:** Creates a new package. When creating a package, you must
specify the package name, build type, and dependencies.

**Format:**

``` bash
ros2 pkg create <package_name> --build-type <build-type> --dependencies <dependencies>
```

**Parameters:**

-   **pkg**: Package-related operations\
-   **create**: Create a new package\
-   **package_name**: Name of the new package\
-   **build-type**: `ament_cmake` for C++, `ament_python` for Python\
-   **dependencies**: Optional dependencies (C++ requires `rclcpp`,
    Python requires `rclpy`)

------------------------------------------------------------------------

### 1.2 `ros2 pkg list`

**Function:** View all packages in the system.

**Format:**

``` bash
ros2 pkg list
```

![ROS2 Package List](/img/docs/jetson/10-ROS2/10-15/image-20230427154146879.png)

------------------------------------------------------------------------

### 1.3 `ros2 pkg executables`

**Function:** View all executable files in a package.

**Format:**

``` bash
ros2 pkg executables pkg_name
```

![ROS2 Package Executables](/img/docs/jetson/10-ROS2/10-15/image-20230427154419005.png)

------------------------------------------------------------------------

## 2. Node Execution Tool: `ros2 run`

**Function:** Run a node program in a specified package.

**Format:**

``` bash
ros2 run pkg_name node_name
```

-   **pkg_name**: Package name\
-   **node_name**: Executable name

![ROS2 Run Node](/img/docs/jetson/10-ROS2/10-15/image-20230427155359745.png)

------------------------------------------------------------------------

## 3. Node-Related Tools: `ros2 node`

### 3.1 `ros2 node list`

**Function:** List all nodes in the current domain.

**Format:**

``` bash
ros2 node list
```

![ROS2 Node List](/img/docs/jetson/10-ROS2/10-15/image-20230427155655753.png)

------------------------------------------------------------------------

### 3.2 `ros2 node info`

**Function:** View detailed node information (publishers, subscribers,
services, actions).

**Format:**

``` bash
ros2 node info node_name
```

![ROS2 Node Info](/img/docs/jetson/10-ROS2/10-15/image-20230427160001101.png)

------------------------------------------------------------------------

## 4. Topic-Related Tools: `ros2 topic`

### 4.1 `ros2 topic list`

**Function:** List all topics.

``` bash
ros2 topic list
```

![ROS2 Topic List](/img/docs/jetson/10-ROS2/10-15/image-20230427160351051.png)

------------------------------------------------------------------------

### 4.2 `ros2 topic info`

**Function:** Show topic type and publisher/subscriber count.

``` bash
ros2 topic info topic_name
```

![ROS2 Topic Info](/img/docs/jetson/10-ROS2/10-15/image-20230427160625594.png)

------------------------------------------------------------------------

### 4.3 `ros2 topic type`

**Function:** Display message type for a topic.

``` bash
ros2 topic type topic_name
```

![ROS2 Topic Type](/img/docs/jetson/10-ROS2/10-15/image-20230427161056536.png)

------------------------------------------------------------------------

### 4.4 `ros2 topic hz`

**Function:** Display publishing frequency of a topic.

``` bash
ros2 topic hz topic_name
```

![ROS2 Topic Frequency](/img/docs/jetson/10-ROS2/10-15/image-20230427161721650.png)

------------------------------------------------------------------------

### 4.5 `ros2 topic echo`

**Function:** Print topic messages in the terminal.

``` bash
ros2 topic echo topic_name
```

![ROS2 Topic Echo](/img/docs/jetson/10-ROS2/10-15/image-20230427162312888.png)

------------------------------------------------------------------------

### 4.6 `ros2 topic pub`

**Function:** Publish a message to a topic from the terminal.

``` bash
ros2 topic pub topic_name message_type message_content
```

**Advanced Options:**

-   `-1`: Publish only once\
-   `-t <count>`: Publish `<count>` times\
-   `-r <hz>`: Publish at `<hz>` frequency

**Example:**

``` bash
ros2 topic pub turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

![ROS2 Topic Publish](/img/docs/jetson/10-ROS2/10-15/image-20250905163149992.png)

------------------------------------------------------------------------

## 5. Interface-Related Tools: `ros2 interface`

### 5.1 `ros2 interface list`

**Function:** List all available interfaces (topics, services, actions).

``` bash
ros2 interface list
```

![ROS2 Interface List](/img/docs/jetson/10-ROS2/10-15/image-20230427164755075.png)

------------------------------------------------------------------------

### 5.2 `ros2 interface show`

**Function:** Display details of a specific interface.

``` bash
ros2 interface show interface_name
```

------------------------------------------------------------------------

> This documentation is maintained by **HemiHex** for educational and
> development purposes.
