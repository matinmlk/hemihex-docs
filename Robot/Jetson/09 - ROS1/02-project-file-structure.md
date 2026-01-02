---
title: Project File Structure
sidebar_position: 2
---

# 2. Project File Structure

## 2.1 Project File Structure

The file structure of ROS is not mandatory for every folder and is designed according to business needs.

```mermaid
flowchart TD
    A[catkin workspace]

    A --> B[build]
    A --> C[src]
    A --> D[devel]
    A --> E["install<br/>(Hardly)"]

    C --> P1[package1]
    C --> P2[package2]
    C --> Pn["..."]

    P2 --> F1[CMakeLists.txt]
    P2 --> F2[package.xml]
    P2 --> F3[scripts]
    P2 --> F4[msg]
    P2 --> F5[srv]
    P2 --> F6[src]
    P2 --> F7[include]
    P2 --> F8[launch]

    F3 --> S1["*.py"]
    F3 --> S2["*.sh"]

    F4 --> M1["*.msg"]
    F5 --> R1["*.srv"]

    F6 --> C1["*.cpp"]
    F7 --> H1["*.h"]
    F8 --> L1["*.launch"]

```
------------------------------------------------------------------------

## 2.2 Workspace

A workspace is a place to manage and organize ROS engineering project files. It can be intuitively understood as a warehouse that contains various ROS projects, making it easy for the system to organize, manage, and call them. In a graphical interface, it is simply a folder. The ROS code we write ourselves is usually stored in the workspace.

There are four main primary directories under a workspace:

- `src` — Source space; ROS Catkin software packages (source code packages)
- `build` — Compilation space; cache information and intermediate files for Catkin (CMake)
- `devel` — Development space; generated target files (including header files, dynamic libraries, static libraries, executable files, etc.), and environment variables
- `install` — Installation space

The top-level workspace (can be named arbitrarily) and the `src` folder (must be named `src`) need to be created manually.

- The `build` and `devel` folders are created automatically by the `catkin_make` command.
- The `install` folder is created by the `catkin_make install` command and is rarely used.

:::note
When using `catkin_make`, make sure to return to the top-level workspace before compiling. In the same workspace, feature packages with the same name are not allowed. Across different workspaces, packages with the same name are allowed.
:::

```bash
mkdir -p ~/catkin_ws/src        # create workspace and src folder
cd catkin_ws                   # enter the workspace
catkin_make                    # compile
source devel/setup.bash        # update the workspace environment
```

## 2.3 Package (Feature Package)

A package is a specific combination of file structures and folders. Usually, program code that implements the same specific function is placed in one package. Only `CMakeLists.txt` and `package.xml` are required; the remaining directories depend on the needs of the software package.

### Create a Feature Package

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_pkg rospy rosmsg roscpp
```

`rospy`, `rosmsg`, and `roscpp` are dependency libraries that can be added according to project needs. Adding them during creation avoids additional configuration later. If you forget to add them, they must be configured manually.

### Typical Package File Structure

```text
|-- CMakeLists.txt
|-- package.xml
|-- include/
|-- config/
|-- launch/
|-- meshes/
|-- urdf/
|-- rviz/
|-- src/
|-- scripts/
|-- srv/
|-- msg/
|-- action/
```

## 2.4 Introduction to `CMakeLists.txt`

### 2.4.1 Overview

`CMakeLists.txt` is the rule file for the CMake compilation system. The Catkin build system follows the same structure as CMake, with additional macros specific to ROS.

This file specifies:

- Which packages this project depends on
- Which targets are compiled and generated
- How compilation and linking are handled

When the Catkin build system runs, it first finds the `CMakeLists.txt` file under each package and then compiles according to those rules.

### 2.4.2 Basic Format

```cmake
cmake_minimum_required()
project()
find_package()
catkin_python_setup()
add_message_files()
add_service_files()
add_action_files()
generate_messages()
catkin_package()
add_library()
add_executable()
add_dependencies()
target_link_libraries()
catkin_add_gtest()
install()
```

### 2.4.3 Boost

```cmake
find_package(Boost REQUIRED COMPONENTS thread)
```

### 2.4.4 `catkin_package()`

```cmake
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet
  DEPENDS eigen opencv
)
```

### 2.4.5 Include & Link Directories

```cmake
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
```

```cmake
link_directories(~/my_libs)
```

:::warning
Using `link_directories()` is generally not recommended. Prefer defining dependencies via `find_package()` and `target_link_libraries()`.
:::
