---
title: Read data
sidebar_position: 2
---

## API Overview

APIs related to reading robotic arm data are as follows

**get_angles()**

Function: Read the angles of the six joints of the robotic arm.

Parameter explanation:

- Return value: Returns six joint angle values.

**get_coords()**

Function: Read the current position coordinate value of the robotic arm.

Parameter explanation:

- Return value: Returns the current position coordinate value of the robotic arm

**get_encoders()**

Function: Read the potential values of the six joints of the robotic arm

Parameter explanation:

- Return value: Return the potential values of the six joints of the robotic arm.

**get_radians()**

Function: Read the arc values of the six joints of the robotic arm.

Parameter explanation:

- Return value: Returns the radian values of the six joints of the robotic arm.

**get_gripper_value()**

Function: Read the angle value of the robotic arm gripper.

Parameter explanation:

- Return value: Returns the angle value of the robotic arm gripper.

## 2. About code

Code path：~/jetcobot_ws/src/jetcobot_ctrl/scripts/read_data.ipynb

```python
#!/usr/bin/env python3
#coding=utf-8
import os
import time
from pymycobot.mycobot import MyCobot
```

Initialize the robotic arm object.

```python
mc = MyCobot(str(os.getenv('MY_SERIAL')), 1000000)
```

Read and print the angles of the six joints of the robotic arm.

```python
angles = mc.get_angles()
print("read angles:", angles)
```

Read and print the coordinates of the robotic arm.

```python
coords = mc.get_coords()
print("read coords:", coords)
```

Read and print the potential value of the robotic arm.

```python
encoders = mc.get_encoders()
print("encoders:", encoders)
```

Read and print the arc value of the robotic arm.

```python
radians = mc.get_radians()
print("read radians:", radians)
```

Read and print the gripper angle of the robotic arm.

```python
gripper = mc.get_gripper_value()
print("gripper:", gripper)
```

## 3.Run program

Click the run button on jupyterlab to run the relevant programs.

![image-20240701162416069](/img/docs/hhbot/11-robotic-arm-control-course/11-2/image-20240701162416069.png)
