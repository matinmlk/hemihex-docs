---
sidebar_position: 24
title: ROS 2 DDS
---

# 14. ROS 2 DDS

## 1. Introduction to DDS

DDS stands for **Data Distribution Service**. It is a real-time,
data-centric publish/subscribe communication standard released by the
**Object Management Group (OMG)** in 2004.

DDS was initially adopted by the U.S. Navy to address large-scale
distributed system communication challenges and has since become a
widely used real-time communication standard.

DDS emphasizes:

-   Data-centric communication
-   High-performance real-time transmission
-   Flexible **Quality of Service (QoS)** control

### References

-   Fast DDS Documentation:\
    https://fast-dds.docs.eprosima.com/en/latest/

-   ROS 2 DDS Advanced Functions:\
    https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html

------------------------------------------------------------------------

## 2. Communication Model

Topics, services, and actions in ROS 2 are all implemented on top of
DDS. DDS acts as the **communication backbone** of the ROS 2 system.

Four common communication models are:

### 1. Peer-to-Peer Model

Each client directly connects to the server. As the number of nodes
increases, the number of connections increases rapidly.

**Disadvantages:** - Poor scalability - Hardcoded server addresses -
High maintenance cost

------------------------------------------------------------------------

### 2. Broker Model

A central broker forwards all messages between nodes.

**Disadvantages:** - Performance bottleneck - Single point of failure

ROS 1 used a similar centralized architecture.

------------------------------------------------------------------------

### 3. Broadcast Model

All nodes broadcast and receive messages on the same channel.

**Disadvantages:** - All nodes receive all messages - Heavy traffic and
filtering overhead

------------------------------------------------------------------------

### 4. DDS Data-Centric Model

DDS uses a **DataBus** where nodes only subscribe to the data they need.

**Advantages:** - Parallel communication paths - High scalability -
Efficient filtering - No central broker

------------------------------------------------------------------------

## 3. Application of DDS in ROS 2

DDS is the **core infrastructure layer** of ROS 2.

All ROS 2 communication mechanisms rely on DDS for:

-   Discovery
-   Transport
-   Reliability
-   Data synchronization

This design allows developers to focus on application logic instead of
networking.

------------------------------------------------------------------------

## 4. Quality of Service (QoS)

### Domain Concept

A **DDS Domain** defines a logical communication space. Only nodes
within the **same domain** can communicate.

This is controlled via:

``` bash
export ROS_DOMAIN_ID=0
```

------------------------------------------------------------------------

### Core QoS Policies

  Policy        Description
  ------------- -----------------------------------------------------
  DEADLINE      Data must be transmitted within a time limit
  HISTORY       Number of historical messages stored
  RELIABILITY   BEST_EFFORT vs RELIABLE
  DURABILITY    Allow late-joining nodes to receive historical data

------------------------------------------------------------------------

## 5. Test Cases

### 5.1 Case 1 --- Command-Line DDS QoS Configuration

#### Publisher Terminal

``` bash
ros2 topic pub /chatter std_msgs/msg/Int32 "data: 66" --qos-reliability best_effort
```

#### Subscriber Terminal (Mismatched QoS)

``` bash
ros2 topic echo /chatter --qos-reliability reliable
```

⚠️ A warning will appear due to mismatched QoS.

#### Correct Matching QoS

``` bash
ros2 topic echo /chatter --qos-reliability best_effort
```

✅ Data will now be received correctly.

------------------------------------------------------------------------

### 5.2 Case 2 --- Configuring QoS in Topic Nodes

#### Step 1: Create Package

``` bash
ros2 pkg create learning_dds --build-type ament_python --dependencies rclpy std_msgs
```

------------------------------------------------------------------------

### Publisher Node (`dds_controller_pub.py`)

``` python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ControllerPublisher(Node):

    def __init__(self, name):
        super().__init__(name)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            String,
            "/robot_cmd",
            self.qos_profile
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cmd_list = ["forward", "backward", "stop"]
        self.cmd_index = 0

    def timer_callback(self):
        current_cmd = self.cmd_list[self.cmd_index % 3]

        msg = String()
        msg.data = current_cmd

        self.publisher.publish(msg)
        self.get_logger().info(f"Command Sent: {msg.data}")

        self.cmd_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControllerPublisher("robot_controller_pub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

------------------------------------------------------------------------

✅ This completes DDS-based QoS-controlled communication in ROS 2.
