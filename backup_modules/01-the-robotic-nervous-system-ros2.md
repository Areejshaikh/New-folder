---
title: "The Robotic Nervous System (ROS 2)"
sidebar_label: "ROS 2 Basics"
description: "Introduction to Robot Operating System 2 (ROS 2) - the middleware that connects all parts of a robot system"
---

# The Robotic Nervous System (ROS 2)

## Introduction

Robot Operating System 2 (ROS 2) is the next generation of the popular robotic middleware. Though not an operating system itself, ROS 2 provides libraries and tools to help software developers create robotic applications. Think of ROS 2 as the nervous system of a robot, connecting sensors, actuators, and computational processes in a coordinated way.

In this module, we'll explore the fundamentals of ROS 2 and how it serves as the backbone for complex robotic systems, particularly humanoid robots. You'll learn how different components of a robot can communicate, coordinate actions, and share data through the ROS 2 framework.

## Learning Objectives

By the end of this module, you will be able to:

1. Explain the fundamental concepts of ROS 2 architecture (nodes, topics, services)
2. Create and run basic ROS 2 nodes in Python using rclpy
3. Understand how message passing works in ROS 2
4. Implement simple publishers and subscribers
5. Create simple services for request-response communication

## Prerequisites

Before starting this module, you should have:

- Basic Python programming skills (variables, functions, classes)
- Understanding of object-oriented programming concepts
- Familiarity with command-line tools
- Completed the setup phase of this curriculum

Estimated duration: 20 hours

## Table of Contents

1. [ROS 2 Architecture Overview](#ros-2-architecture-overview)
2. [Setting Up Your ROS 2 Environment](#setting-up-your-ros-2-environment)
3. [Nodes: The Building Blocks](#nodes-the-building-blocks)
4. [Topics and Messages: Streaming Data](#topics-and-messages-streaming-data)
5. [Services: Request-Response Communication](#services-request-response-communication)
6. [Actions: Goal-Oriented Communication](#actions-goal-oriented-communication)
7. [Hands-On Exercises](#hands-on-exercises)
8. [Summary and Next Steps](#summary-and-next-steps)

---

## ROS 2 Architecture Overview

![ROS 2 Architecture](/img/ros2-architecture.png)

ROS 2 is composed of several core elements that work together to enable complex robotic systems:

### Nodes
Nodes are the fundamental units of computation in ROS 2. Each node is typically responsible for a specific function in the robot system, such as sensor processing, motion control, or decision making. Nodes communicate with each other through topics, services, and actions.

### Topics
Topics enable asynchronous communication between nodes using a publish-subscribe model. Multiple nodes can publish messages to the same topic, and multiple nodes can subscribe to the same topic. This is ideal for streaming data like sensor readings or actuator commands.

### Services
Services provide synchronous request-response communication. A service client sends a request to a service server, which then processes the request and returns a response. This is useful for operations that need to complete before the client proceeds.

### Actions
Actions are similar to services but designed for long-running tasks. They provide feedback during execution and can be preempted. Actions are ideal for navigation, manipulation, and other tasks that take time and may need to be monitored or interrupted.

### Packages
Packages are the basic building units of code in ROS 2. A package can contain one or more nodes, along with configuration files, launch files, and other resources.

## Setting Up Your ROS 2 Environment

Before diving into ROS 2 programming, you'll need to set up your development environment. If you haven't already, ensure that you have installed ROS 2 Humble Hawksbill (the LTS version we're using in this curriculum).

### Source the ROS 2 Environment

Every time you open a new terminal for ROS 2 development, you need to source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash  # On Linux
# Or if using Windows with WSL:
source /opt/ros/humble/setup.bash
```

### Create a Workspace

ROS 2 uses workspaces to organize packages. Let's create a workspace for our robotics learning:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Later in this curriculum, we'll create packages in the `src` directory and build them using the `colcon` build tool.

## Nodes: The Building Blocks

Nodes are individual programs that perform computations. In a humanoid robot system, you might have nodes for:

- Joint controllers
- Sensor data processing (IMU, cameras, lidar)
- Path planning
- Navigation
- Task management
- High-level decision making

Let's create our first ROS 2 node in Python to understand the basics.

### Creating a Simple Node

First, let's create the directory structure for our simple node package:

```bash
mkdir -p ~/ros2_ws/src/my_robot_tutorials
cd ~/ros2_ws/src/my_robot_tutorials
mkdir -p my_robot_tutorials/my_robot_tutorials
touch my_robot_tutorials/__init__.py
```

Now, let's create a simple publisher node that sends a counter value periodically:

```python
# my_robot_tutorials/my_robot_tutorials/simple_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(Int64, 'counter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    # Destroy the node explicitly
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

And create a corresponding subscriber node that listens to the counter:

```python
# my_robot_tutorials/my_robot_tutorials/simple_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'counter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Package Configuration

To run these nodes as ROS 2 packages, we also need a `setup.py` file to make the scripts runnable:

```python
# setup.py
from setuptools import find_packages, setup

package_name = 'my_robot_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='curriculum',
    maintainer_email='curriculum@example.com',
    description='ROS 2 tutorials for the PAHR book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_tutorials.simple_publisher:main',
            'simple_subscriber = my_robot_tutorials.simple_subscriber:main',
        ],
    },
)
```

And a `package.xml` file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_tutorials</name>
  <version>0.0.0</version>
  <description>ROS 2 tutorials for the PAHR book</description>
  <maintainer email="curriculum@example.com">curriculum</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Topics and Messages: Streaming Data

Topics implement one of the fundamental communication patterns in ROS 2: publish-subscribe. A publisher node sends messages to a topic, and subscriber nodes receive those messages. Multiple nodes can listen to the same topic, and multiple nodes can send messages to the same topic.

Message types define the structure of data exchanged between nodes. ROS 2 comes with standard message types (like `std_msgs/Int64` we used above), but we can also define custom message types.

### Message Types

Common message types in ROS 2 include:

- `std_msgs`: Basic data types (Bool, Int64, Float64, String, etc.)
- `geometry_msgs`: 3D geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs`: Data types for common sensors (Image, LaserScan, JointState, etc.)
- `nav_msgs`: Data types for navigation (Odometry, Path, OccupancyGrid, etc.)

### Creating Message Publishers and Subscribers

The nodes we created in the previous section demonstrate the basic pattern of creating a publisher and subscriber for a topic. When we run:

```bash
ros2 run my_robot_tutorials simple_publisher
```

And in another terminal:

```bash
ros2 run my_robot_tutorials simple_subscriber
```

The publisher will send integer values to the `/counter` topic, and the subscriber will receive and log these values.

## Services: Request-Response Communication

Services provide synchronous communication between nodes. Unlike topics, which are asynchronous and unidirectional, services follow a request-response pattern where a client sends a request and waits for a response from a server.

### Creating a Service Server

Here's an example of a simple service that adds two numbers:

```python
# my_robot_tutorials/my_robot_tutorials/add_two_ints_server.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()

    rclpy.spin(add_two_ints_server)

    # Destroy the node explicitly
    add_two_ints_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Service Client

And a client that calls the service:

```python
# my_robot_tutorials/my_robot_tutorials/add_two_ints_client.py

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    add_two_ints_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    add_two_ints_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions: Goal-Oriented Communication

Actions are used for long-running tasks that provide feedback and can be cancelled. Unlike services which are synchronous and complete immediately, and topics which are asynchronous, actions are synchronous in the sense that they expect to complete, but they provide feedback during execution.

An action has three components:
- **Goal**: What the action should do
- **Feedback**: Information about progress toward the goal
- **Result**: The final outcome of the action

## Hands-On Exercises

### Exercise 1: Publisher-Subscriber Pair

Create two nodes that communicate via a custom message topic. The publisher should send sensor data (e.g., temperature readings) and the subscriber should log these readings and detect if they exceed a threshold.

### Exercise 2: Service Implementation

Implement a service that calculates the distance between two points in 3D space. Create both the server and client nodes, and test the communication between them.

### Exercise 3: Custom Message

Define your own message type for a humanoid robot joint state (containing joint name, position, velocity, and effort) and use it in a publisher-subscriber pair.

## Summary and Next Steps

In this module, we've introduced the core concepts of ROS 2, including nodes, topics, services, and the overall architecture. We've created a simple publisher-subscriber pair and a service client-server pair to demonstrate how different components of a robotic system can communicate.

In the next module, we'll dive deeper into how to connect Python agents to ROS 2 controllers, which is a crucial skill for implementing AI algorithms that interact with robotic systems directly.

## Key Takeaways

- ROS 2 provides a middleware layer that connects different components of a robot system
- Nodes are the fundamental units of computation in ROS 2
- Topics enable asynchronous communication through publish-subscribe
- Services provide synchronous request-response communication
- Actions are for long-running operations with feedback and cancellation support
- Each communication method has its appropriate use cases in robotic systems

## Self-Assessment Questions

1. What are the main differences between topics, services, and actions in ROS 2?
2. Why is the publish-subscribe model beneficial for sensor data in robotic systems?
3. When would you use a service instead of a topic for communication between nodes?
4. How do different programming languages communicate in ROS 2?

---

<nav>
  <div class="prev-next-pagination" layout="row" align="center" justify="between">
    <a href="/docs/intro"><- Previous: Introduction</a>
    <a href="/docs/modules/02-bridging-python-agents">Next: Bridging Python Agents to ROS Controllers -></a>
  </div>
</nav>