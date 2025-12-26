---
title: Subscribers
sidebar_position: 5
---

# 5. Subscribers

## 5.1 Subscribers

The subscriber receives the data published by the publisher and then enters its callback function, where the received data is processed. The core logic of a subscriber is the **callback function**, which is executed whenever a new message arrives on a subscribed topic.

---

## 5.2 Create a Subscriber

### 5.2.1 Creation Steps

1. Initialize the ROS node  
2. Create a node handle  
3. Subscribe to the required topic  
4. Wait for topic messages in a loop, and upon receiving the message, enter the callback function  
5. Complete message processing inside the callback function  

---

## 5.2.2 C++ Language Implementation

```text
turtle_pose_subscriber.cpp
```

```cpp
#include <ros/ros.h>
#include "turtlesim/Pose.h"

void turtle_poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("Turtle pose: x:%0.3f, y:%0.3f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_pose_subscriber");
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, turtle_poseCallback);

    ros::spin();

    return 0;
}
```

![C++ Subscriber Flow](/img/docs/jetson/9-ROS1/9-5/sub_c++.jpg)

```cmake
add_executable(turtle_pose_subscriber src/turtle_pose_subscriber.cpp)
target_link_libraries(turtle_pose_subscriber ${catkin_LIBRARIES})
```

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun learning_topic turtle_pose_subscriber
```

![C++ Subscriber Output](/img/docs/jetson/9-ROS1/9-5/image-20220225102205852.png)

---

## 5.2.3 Python Language Implementation

```text
turtle_pose_subscriber.py
```

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.3f, y:%0.3f", msg.x, msg.y)

def turtle_pose_subscriber():
    rospy.init_node('turtle_pose_subscriber', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
    rospy.spin()

if __name__ == '__main__':
    turtle_pose_subscriber()
```

![Python Subscriber Flow](/img/docs/jetson/9-ROS1/9-5/image-20220225103401539.png)

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun learning_topic turtle_pose_subscriber
```

---

## Summary

This section demonstrated how to implement ROS subscribers using both **C++** and **Python**, subscribe to the `/turtle1/pose` topic, and continuously receive and process turtle pose data through callback functions.
