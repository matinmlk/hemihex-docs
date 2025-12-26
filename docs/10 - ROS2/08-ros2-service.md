---
sidebar_position: 8
title: ROS 2 Service Communication
---

# ROS 2 Service Communication

## 1. Introduction

Service communication is a **request--response** communication model.
The client sends a request, and the server processes it and returns a
response.

------------------------------------------------------------------------

## 2. Create a New Package

``` bash
ros2 pkg create pkg_service --build-type ament_python --dependencies rclpy --node-name server_demo
```

------------------------------------------------------------------------

## 3. Server Implementation

### 3.1 Create the Server Node

Edit `server_demo.py`:

``` python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Server(Node):

    def __init__(self, name):
        super().__init__(name)

        self.srv = self.create_service(
            AddTwoInts,
            '/add_two_ints',
            self.Add2Ints_callback
        )

    def Add2Ints_callback(self, request, response):
        response.sum = request.a + request.b
        print("response.sum =", response.sum)
        return response

def main():
    rclpy.init()
    server_demo = Service_Server("publisher_node")
    rclpy.spin(server_demo)
    server_demo.destroy_node()
    rclpy.shutdown()
```

------------------------------------------------------------------------

### 3.2 Inspect the Service Interface

``` bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

------------------------------------------------------------------------

### 3.3 Register the Server in `setup.py`

``` python
'server_demo = pkg_service.server_demo:main',
```

------------------------------------------------------------------------

### 3.4 Build the Package

``` bash
colcon build --packages-select pkg_service
```

------------------------------------------------------------------------

### 3.5 Run the Server

``` bash
ros2 run pkg_service server_demo
```

------------------------------------------------------------------------

### 3.6 Call the Service

``` bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 4}"
```

------------------------------------------------------------------------

## 4. Client Implementation

### 4.1 Create the Client File

Create `client_demo.py`.

------------------------------------------------------------------------

### 4.2 Client Implementation Code

``` python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Client(Node):

    def __init__(self, name):
        super().__init__(name)

        self.client = self.create_client(AddTwoInts, '/add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            print("Service not available, waiting...")

        self.request = AddTwoInts.Request()
        self.request.a = 10
        self.request.b = 20

        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            print("Service response:", response.sum)
        except Exception as e:
            print("Service call failed:", e)

def main():
    rclpy.init()
    client_demo = Service_Client("client_node")
    rclpy.spin(client_demo)
    client_demo.destroy_node()
    rclpy.shutdown()
```

------------------------------------------------------------------------

### 4.3 Register the Client in `setup.py`

``` python
'client_demo = pkg_service.client_demo:main',
```

------------------------------------------------------------------------

### 4.4 Rebuild the Package

``` bash
colcon build --packages-select pkg_service
```

------------------------------------------------------------------------

### 4.5 Run the Client

Ensure the server is running first:

``` bash
ros2 run pkg_service server_demo
```

Then run the client:

``` bash
ros2 run pkg_service client_demo
```

------------------------------------------------------------------------

Maintained by **HemiHex**.
