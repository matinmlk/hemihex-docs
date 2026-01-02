---
sidebar_position: 9
title: Custom Service Messages and Usage
---

# 9. Custom Service Messages and Usage

This section explains how to define and use **custom ROS service
messages**, including configuration, compilation, and both C++
server/client implementations.

------------------------------------------------------------------------

## 9.1 Customized Service Messages

Switch to the following directory and create a new folder named `srv` to
store custom service message files:

``` bash
~/catkin_ws/src/learning_server/srv
```

------------------------------------------------------------------------

### 9.1.1 Define SRV Files

Create a new `.srv` file. Here we use **IntPlus.srv** as an example:

``` text
uint8  a
uint8  b

---

uint8 result
```

**Explanation:**\
The SRV file is divided into two parts by the `---` separator:

-   **Top section** → Request data\
-   **Bottom section** → Response data

------------------------------------------------------------------------

### 9.1.2 Add Feature Pack Dependencies in `package.xml`

Add the following dependencies:

``` xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

------------------------------------------------------------------------

### 9.1.3 Add Compilation Options in `CMakeLists.txt`

Add the following configuration:

``` cmake
add_service_files(FILES IntPlus.srv)
generate_messages(DEPENDENCIES std_msgs)
```

------------------------------------------------------------------------

### 9.1.4 Compile and Generate Language-Related Files

``` bash
cd ~/catkin_ws
catkin_make
```

------------------------------------------------------------------------

## 9.1.5 C++ Language Implementation

### Step 1: Create Server and Client Source Files

Navigate to:

``` bash
~/catkin_ws/src/learning_server/src
```

Create the following files:

``` text
IntPlus_server.cpp
IntPlus_client.cpp
```

------------------------------------------------------------------------

### IntPlus_server.cpp

``` cpp
#include <ros/ros.h>
#include "learning_server/IntPlus.h"

bool IntPlusCallback(learning_server::IntPlus::Request  &req,
                     learning_server::IntPlus::Response &res)
{
    ROS_INFO("number 1 is:%d , number 2 is:%d", req.a, req.b);

    res.result = req.a + req.b;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IntPlus_server");
    ros::NodeHandle n;

    ros::ServiceServer Int_Plus_service =
        n.advertiseService("/Two_Int_Plus", IntPlusCallback);

    ROS_INFO("Ready to calculate.");
    ros::spin();

    return 0;
}
```

------------------------------------------------------------------------

### IntPlus_client.cpp

``` cpp
#include <ros/ros.h>
#include "learning_server/IntPlus.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    int i, k;
    cin >> i;
    cin >> k;

    ros::init(argc, argv, "IntPlus_client");
    ros::NodeHandle node;

    ros::service::waitForService("/Two_Int_Plus");

    ros::ServiceClient IntPlus_client =
        node.serviceClient<learning_server::IntPlus>("/Two_Int_Plus");

    learning_server::IntPlus srv;
    srv.request.a = i;
    srv.request.b = k;

    ROS_INFO("Call service to plus %d and %d",
             srv.request.a, srv.request.b);

    IntPlus_client.call(srv);

    ROS_INFO("Show the result : %d", srv.response.result);

    return 0;
}
```

------------------------------------------------------------------------

### Step 2: Modify `CMakeLists.txt`

``` cmake
add_executable(IntPlus_server src/IntPlus_server.cpp)
target_link_libraries(IntPlus_server ${catkin_LIBRARIES})
add_dependencies(IntPlus_server ${PROJECT_NAME}_generate_messages_cpp)

add_executable(IntPlus_client src/IntPlus_client.cpp)
target_link_libraries(IntPlus_client ${catkin_LIBRARIES})
add_dependencies(IntPlus_client ${PROJECT_NAME}_generate_messages_cpp)
```

------------------------------------------------------------------------

### Step 3: Core Header Import

To use the custom service:

``` cpp
#include "learning_server/IntPlus.h"
```

`learning_server` is the name of the package, and `IntPlus.h` is
auto-generated from the `.srv` file.

------------------------------------------------------------------------

✅ You now have a fully working **custom ROS service** with both server
and client implementations.
