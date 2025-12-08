---
sidebar_position: 14
title: ROS 2 Function Packages
---

# 5. ROS 2 Function Packages

## 1. Introduction to Function Packages

Each robot may have many functions, such as motion control, visual
perception, and autonomous navigation. While it is possible to combine
all source code into one project, sharing and maintaining such code
becomes difficult.

Function packages solve this by separating different functional modules
into independent packages. This minimizes coupling and greatly improves
**code reuse** and **collaboration** within the ROS community.

------------------------------------------------------------------------

## 2. Creating Function Packages

To create a function package in ROS 2, use the following command format:

``` bash
ros2 pkg create <package_name> --build-type <build-type> --dependencies <dependencies> --node-name <node-name>
```

### Command Parameter Description

-   **pkg**: Operates on ROS 2 packages\
-   **create**: Creates a new package\
-   **package_name**: Required, name of the package\
-   **build-type**: Required
    -   `ament_cmake` → C++ packages\
    -   `ament_python` → Python packages\
-   **dependencies**: Optional, such as:
    -   `rclcpp` for C++
    -   `rclpy` for Python\
-   **node-name**: Optional, auto-generates an executable node

------------------------------------------------------------------------

### Switch to Workspace Source Directory

Replace `workspace` with your actual workspace path:

``` bash
cd workspace/src
```

------------------------------------------------------------------------

### Create a C++ Package Example

``` bash
ros2 pkg create pkg_helloworld_cpp --build-type ament_cmake --dependencies rclcpp --node-name helloworld
```

------------------------------------------------------------------------

### Create a Python Package Example

``` bash
ros2 pkg create pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
```

------------------------------------------------------------------------

## 3. Compile the Package

After writing code inside the package, compile the workspace:

### Compile All Packages

``` bash
colcon build
```

------------------------------------------------------------------------

### Compile Specific Packages

``` bash
colcon build --packages-select pkg1 pkg2
```

------------------------------------------------------------------------

## 4. Complete Workspace Structure with Feature Packages

The standard ROS 2 workspace structure is shown below:

``` text
WorkSpace
 |--- build        # Build output for each package
 |--- install      # Installed packages
 |--- log          # Build logs
 |--- src          # Source directory
       |-- C++ Package
       |     |-- package.xml
       |     |-- CMakeLists.txt
       |     |-- src
       |     |-- include
       |     |-- msg
       |     |-- srv
       |     |-- action
       |
       |-- Python Package
             |-- package.xml
             |-- setup.py
             |-- setup.cfg
             |-- resource
             |-- test
             |-- <package_name>
```

------------------------------------------------------------------------

### Optional Configuration Directories

Both C++ and Python packages may also include:

``` text
|-- launch    # Launch files
|-- rviz      # RViz configurations
|-- urdf      # Robot model files
|-- params    # Parameter files
|-- world     # Simulation environments
|-- map       # Navigation maps
|-- ...
```

These directories can be customized or extended based on project
requirements.
