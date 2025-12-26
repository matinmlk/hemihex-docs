---
sidebar_position: 31
title: ROS 2 URDF Model
---

# ROS 2 URDF Model

## 1. Introduction to URDF

The modeling method in ROS is called **URDF**, which stands for
**Unified Robot Description Format**. It is used to describe:

-   The robot model itself\
-   The robot's external environment

URDF model files use the **XML format**.

External reference:

-   https://wiki.ros.org/sw_urdf_exporter

------------------------------------------------------------------------

## 2. Robot Components

When modeling and describing a robot, we must first understand its
components and parameters. A robot is generally composed of four major
systems:

-   **Hardware structure**
-   **Drive system**
-   **Sensor system**
-   **Control system**

This applies to both mobile robots and robotic arms.

### Component Breakdown

-   **Hardware Structure** --- chassis, housing, motors\
-   **Drive System** --- motor drivers, power electronics\
-   **Sensor System** --- IMU, cameras, LiDAR, encoders\
-   **Control System** --- embedded computer, OS, middleware

The robot modeling process follows the same idea: each component is
described using a modeling language and then assembled into a complete
system.

------------------------------------------------------------------------

## 3. URDF Syntax

### 3.1 Link Description

The `<link>` tag describes a robot's rigid body:

-   Appearance (visual)
-   Physical properties (inertia, mass)
-   Collision model

The `name` attribute defines the link name.

#### Visual Section

-   `<geometry>` --- Defines shape\
-   `<mesh>` --- Loads STL model\
-   `<origin>` --- Position and orientation offset

#### Collision Section

-   Used for physical interaction\
-   Often simplified to reduce computation

Difference:

-   `<visual>` → Appearance only\
-   `<collision>` → Physics calculations

------------------------------------------------------------------------

### 3.2 Joint Description

Rigid bodies are connected using **joints**.

Supported joint types:

1.  **Continuous** --- Infinite rotation (e.g., wheels)\
2.  **Revolute** --- Limited rotation (robot joints)\
3.  **Prismatic** --- Linear sliding\
4.  **Fixed** --- No movement\
5.  **Floating** --- Free 6-DOF motion\
6.  **Planar** --- Planar motion

Each joint includes:

-   `<parent>`
-   `<child>`
-   `<origin>`
-   `<axis>`
-   `<limit>`

------------------------------------------------------------------------

## 4. Complete Robot Model Structure

All `<link>` and `<joint>` elements are placed inside a `<robot>` root
tag.

### Recommended Reading Order

1.  Identify all **links**\
2.  Identify all **joints**\
3.  Understand full structure\
4.  Analyze parameter values

------------------------------------------------------------------------

## 5. Creating a Robot Model

Using a mobile robot model as an example, copy the robot description
package into your workspace `src` directory.

### Directory Structure

-   `urdf` --- URDF/XACRO files\
-   `meshes` --- STL mesh files\
-   `launch` --- Launch files\
-   `rviz` --- RViz display configs

------------------------------------------------------------------------

## 6. Compile the Description Package

``` bash
colcon build --packages-select hemihexcar_description
```

------------------------------------------------------------------------

## 7. Model Visualization in RViz

Refresh environment and launch:

``` bash
ros2 launch hemihexcar_description display.launch.py
```

RViz will automatically start and display the robot model.

------------------------------------------------------------------------

## 8. Development Workflow

Typical URDF workflow:

1.  Define links\
2.  Define joints\
3.  Load meshes\
4.  Verify collisions\
5.  Visualize in RViz\
6.  Adjust TF tree alignment

------------------------------------------------------------------------

This documentation is maintained by **HemiHex** for ROS 2 robot modeling
and URDF development workflows.
