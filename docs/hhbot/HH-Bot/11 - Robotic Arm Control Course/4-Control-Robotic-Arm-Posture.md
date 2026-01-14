---
title: Control robotic arm posture
sidebar_position: 4
---

## 1.About code

Code path：~/jetcobot_ws/src/jetcobot_ctrl/scripts/ctrl_poses.ipynb

Create a reset button to control the robotic arm to restore the default posture.

```python
def reset_joints():
mc.send_angles([0, 0, 0, 0, 0, -45], 50)
mc.set_gripper_value(100, 50)
```

Create 10 new robotic arm postures.

```python
def ctrl_poses(id):
if id == 1:
angles = [-2.72, 49.39, -128.67, 76.72, 3.16, -44.2]
mc.send_angles(angles, g_speed)
elif id == 2:
angles = [-84.63, -85.78, -0.61, 93.07, 86.22, -44.2]
mc.send_angles(angles, g_speed)
elif id == 3:
angles = [93.69, -85.78, -0.61, 85.78, -86.13, -44.29]
mc.send_angles(angles, g_speed)
elif id == 4:
angles = [0.26, -40.42, -50.62, 92.98, -0.79, -44.2]
mc.send_angles(angles, g_speed)
elif id == 5:
angles = [0.35, 63.1, -143.26, 73.82, 10.81, -44.73]
mc.send_angles(angles, g_speed)
elif id == 6:
angles = [0, 15, -135, 123, 25, -45]
mc.send_angles(angles, g_speed)
elif id == 7:
angles = [0.43, -0.96, -3.69, 88.76, 10.81, -47.63]
mc.send_angles(angles, g_speed)
elif id == 8:
angles = [97.55, 59.85, -130.25, 70.22, -88.5, -42.01]
mc.send_angles(angles, g_speed)
elif id == 9:
angles = [0.43, 89, -93.95, -36.73, 10.45, -46.23]
mc.send_angles(angles, g_speed)
elif id == 10:
angles = [108.19, -92.1, 125.85, -79.71, 9.74, -43.95]
mc.send_angles(angles, g_speed)
```

## 2.Run program

Click the Run Entire Program button on the Jupyterlab toolbar and scroll to the bottom.

![202309010001](/img/docs/hhbot/11-robotic-arm-control-course/11-4/202309010001.png)

You can see that there are multiple buttons displayed. The robotic arm has ten preset postures.

You can click the corresponding button to make the robotic arm move to the corresponding posture.

![image-20240701171254981](/img/docs/hhbot/11-robotic-arm-control-course/11-4/image-20240701171254981.png)
