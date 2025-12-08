---
sidebar_position: 23
title: ROS 2 Distributed Communication - Domain Rules
---

# ROS 2 Distributed Communication (Part 2: Domain & DDS Rules)

## 1. Distributed Network Grouping (Domain Isolation)

If multiple robots exist on the same network, you can prevent
interference by assigning **different domain IDs**.

Only devices with the **same `ROS_DOMAIN_ID`** can communicate.

Add the following line to the `.bashrc` file on both machines:

``` bash
export ROS_DOMAIN_ID=<your_domain_id>
```

If the host and client use **different values**, they will be isolated
and unable to communicate.

------------------------------------------------------------------------

## 2. Example: Domain-Based Grouping

### Host Machine

``` bash
echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc
source ~/.bashrc
ros2 run demo_nodes_py talker
```

------------------------------------------------------------------------

### Slave Machine

``` bash
echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc
source ~/.bashrc
ros2 run demo_nodes_py listener
```

If the listener receives data correctly, **grouped multi-machine
communication is successful**.

------------------------------------------------------------------------

## 3. ROS_DOMAIN_ID Notes

The `ROS_DOMAIN_ID` value is not arbitrary. The following constraints
apply:

1.  Recommended range: **\[0, 101\]**
2.  Maximum number of nodes per domain: **≤ 120**
3.  For **domain ID 101**, the maximum number of nodes is **54**

------------------------------------------------------------------------

## 4. DDS Domain ID Calculation Rules (Advanced)

1.  DDS communicates using **TCP/IP or UDP/IP**, which use port numbers:

``` text
0 – 65535
```

2.  DDS uses **port 7400 as the starting port**, and each domain
    occupies **250 ports** by default:

``` text
(65535 - 7400) / 250 = 232 domain IDs → [0 – 231]
```

3.  Operating systems reserve some ports:

-   **Linux:** \[0--101\] and \[215--231\]
-   **Windows / macOS:** \[0--166\]

For cross-platform compatibility, **\[0--101\] is the safest range**.

------------------------------------------------------------------------

## 5. Node Capacity per Domain

Each ROS 2 node requires **2 ports**. The first few ports are reserved
for discovery.

``` text
(250 - 10) / 2 = 120 nodes
```

Special case:

-   For `ROS_DOMAIN_ID = 101`, half of the ports are reserved, reducing
    the max to **54 nodes**.

These rules are important for **large-scale robotic systems and
multi-robot network planning**.
