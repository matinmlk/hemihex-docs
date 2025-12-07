---
sidebar_position: 6
title: Customized Topic Messages and Usage
---

# 6. Customized Topic Messages and Usage

## 6.1 Customized Topic Messages

Switch to `~/catkin_ws/src/learning_topic` and create a new folder named
`msg` under the topic feature package directory to store custom topic
messages.

------------------------------------------------------------------------

### 6.1.1 Define `msg` Files

Switch to the `msg` directory and create a new blank `.msg` file. Here
we use `Information.msg` as an example:

``` text
string company
string city
```

------------------------------------------------------------------------

### 6.1.2 Add Feature Package Dependencies in `package.xml`

Add the following dependencies:

``` xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

------------------------------------------------------------------------

### 6.1.3 Add Compilation Options in `CMakeLists.txt`

Add the following:

``` cmake
add_message_files(
  FILES
  Information.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

------------------------------------------------------------------------

### 6.1.4 Compile and Generate Language Files

``` bash
cd ~/catkin_ws
catkin_make
```

------------------------------------------------------------------------

### 6.1.5 C++ Language Implementation

Switch to `~/catkin_ws/src/learning_topic/src` and create two files:

-   `Information_publisher.cpp`
-   `Information_subscriber.cpp`

#### Information Publisher

``` cpp
#include <ros/ros.h>
#include "learning_topic/Information.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "company_information_publisher");
    ros::NodeHandle n;

    ros::Publisher info_pub = n.advertise<learning_topic::Information>("/company_info", 10);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        learning_topic::Information info_msg;
        info_msg.company = "HemiHex";
        info_msg.city = "Toronto";

        info_pub.publish(info_msg);

        ROS_INFO("Information: company: %s  city: %s",
                 info_msg.company.c_str(), info_msg.city.c_str());

        loop_rate.sleep();
    }

    return 0;
}
```

------------------------------------------------------------------------

#### Information Subscriber

``` cpp
#include <ros/ros.h>
#include "learning_topic/Information.h"

void CompanyInfoCallback(const learning_topic::Information::ConstPtr& msg)
{
    ROS_INFO("This is: %s in %s", msg->company.c_str(), msg->city.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "company_information_subscriber");
    ros::NodeHandle n;

    ros::Subscriber info_sub = n.subscribe("/company_info", 10, CompanyInfoCallback);

    ros::spin();
    return 0;
}
```

------------------------------------------------------------------------

### Modify `CMakeLists.txt`

Add the following:

``` cmake
add_executable(Information_publisher src/Information_publisher.cpp)
target_link_libraries(Information_publisher ${catkin_LIBRARIES})
add_dependencies(Information_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(Information_subscriber src/Information_subscriber.cpp)
target_link_libraries(Information_subscriber ${catkin_LIBRARIES})
add_dependencies(Information_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```

------------------------------------------------------------------------

### Core Header Import

``` cpp
#include "learning_topic/Information.h"
```

This header is auto-generated from the custom `.msg` file and enables
both publisher and subscriber to use the custom message type.
