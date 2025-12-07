# Chapter 1: Introduction to ROS 2 Middleware

## Overview
This chapter introduces the fundamental concepts of ROS 2 (Robot Operating System 2) middleware. ROS 2 provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It's designed to support the development of complex robotic applications with improved performance and scalability compared to its predecessor.

## Learning Objectives
By the end of this chapter, you will:
- Understand the fundamental concepts of ROS 2 architecture
- Be familiar with Nodes, Topics, and Services
- Know how to create simple publisher and subscriber systems
- Be able to explain the relationships between different ROS 2 components

## Table of Contents
1. [Introduction to ROS 2](#introduction-to-ros2)
2. [Nodes](#nodes)
3. [Topics](#topics)
4. [Services](#services)
5. [ROS 2 Architecture Relationships](#ros2-architecture-relationships)
6. [Practical Exercise](#practical-exercise)
7. [Summary](#summary)

## Introduction to ROS2
ROS 2 is a collection of software frameworks for developing robot applications. It provides libraries, tools, and conventions that facilitate building, running, and testing robot applications. The middleware layer in ROS 2 handles communication patterns between different components of a robot system.

ROS 2 addresses several limitations of the original ROS:
- Improved real-time performance
- Better security with authentication and encryption
- Platform independence (Windows, Linux, macOS)
- Standardized deployment and lifecycle management

## Nodes
A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. You can group multiple nodes to compose your robot application. Nodes written in different programming languages can be used together in the same system.

### Key Characteristics of Nodes:
- Each node performs a specific function within the robot system
- Nodes communicate with each other through messages
- Nodes can be written in different programming languages
- Nodes can be distributed across multiple machines

### Example Node Structure:
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics
Topics are named buses over which nodes exchange messages. Topics use a publish/subscribe communication pattern where multiple publishers can send messages to the same topic and multiple subscribers can receive messages from the same topic. This allows for loose coupling between nodes.

### Key Characteristics of Topics:
- Unidirectional communication from publishers to subscribers
- Many-to-many relationship (multiple publishers and subscribers)
- Topics are identified by unique names
- Message types are standardized to ensure compatibility

### Topic Communication Pattern:
```
Publisher Node 1 → Topic → Subscriber Node 1
Publisher Node 2         → Subscriber Node 2
                       → Subscriber Node 3
```

## Services
Services provide a request/reply communication pattern between nodes. In this pattern, a node (the client) sends a request message to another node (the server), which then sends back a response message. This synchronous communication is useful for operations that require a specific response.

### Key Characteristics of Services:
- Synchronous request/reply communication
- One-to-one relationship between client and server
- Request and response message types are defined in service files
- Blocking communication (client waits for response)

### Service Communication Pattern:
```
Client Node → Service Request → Server Node
              ← Service Response ←
```

## ROS 2 Architecture Relationships
The ROS 2 architecture consists of several interconnected components:

- **Nodes**: Individual processes that perform computation
- **Topics**: Communication channels for asynchronous message passing
- **Services**: Communication channels for synchronous request/reply
- **Parameters**: Configuration values shared between nodes
- **Actions**: Communication for long-running tasks with feedback

These components interact to form a distributed system where each node can specialize in specific functionality while communicating with others through standardized interfaces.

## Practical Exercise
Now we'll create a simple publisher and subscriber example to demonstrate the concepts we've learned. Follow this step-by-step tutorial to implement your first ROS 2 communication pattern:

### Publisher Node Tutorial
First, let's look at the publisher node code (`simple-publisher.py`):

```python
#!/usr/bin/env python3

"""
Simple ROS 2 publisher example.

This node publishes messages to a topic called 'chatter' at a rate of 10Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """A simple ROS 2 publisher node."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'chatter' topic
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Set a timer to publish messages at a rate of 10Hz (every 0.1 seconds)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for the messages
        self.i = 0

    def timer_callback(self):
        """Callback function for the timer that publishes messages."""
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function."""
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the publisher node
    minimal_publisher = MinimalPublisher()

    # Keep the node running until it's shut down
    rclpy.spin(minimal_publisher)

    # Clean up when done
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node Tutorial
Now, let's implement the subscriber node (`simple-subscriber.py`):

```python
#!/usr/bin/env python3

"""
Simple ROS 2 subscriber example.

This node subscribes to messages from the 'chatter' topic and logs them to the console.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A simple ROS 2 subscriber node."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('minimal_subscriber')

        # Create a subscription that listens to String messages on the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)  # Queue size of 10 messages

        # Make sure the subscription is properly created
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received.

        Args:
            msg: The received message of type std_msgs.msg.String
        """
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function."""
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the subscriber node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running until it's shut down
    rclpy.spin(minimal_subscriber)

    # Clean up when done
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Example
To run these nodes:

1. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Run the publisher node in one terminal:
   ```bash
   python3 simple-publisher.py
   ```

3. Run the subscriber node in another terminal:
   ```bash
   python3 simple-subscriber.py
   ```

4. You should see the publisher sending messages and the subscriber receiving them.

### Key Points from the Example:
- The publisher creates a publisher object using `self.create_publisher()` to send messages
- The subscriber creates a subscription using `self.create_subscription()` to receive messages
- Both nodes use the same topic name ('chatter') for communication
- The publisher sends a message every 0.1 seconds using a timer
- When the subscriber receives a message, the `listener_callback` function is called

## Exercises for Practicing ROS 2 Concepts
1. Modify the publisher to send different types of messages (e.g., integers or custom messages)
2. Create a publisher that sends messages at varying intervals
3. Add error handling to the subscriber to handle malformed messages
4. Experiment with different queue sizes for the publisher and subscriber
5. Create a third node that acts as both a publisher and subscriber, relaying messages between topics

## Summary
This chapter introduced the core concepts of ROS 2 architecture. You learned about Nodes, Topics, and Services, which form the foundation of ROS 2 communication. Through the practical example, you implemented your first publisher and subscriber nodes, demonstrating the publish/subscribe communication pattern. In the next chapter, we'll explore how to bridge Python agents with ROS 2 using the rclpy library.

---

## Advanced Notes (Optional)
- Quality of Service (QoS) settings can be configured for topics to handle different network conditions
- Composition allows multiple nodes to run within the same process for improved performance
- ROS 2 uses Data Distribution Service (DDS) as the underlying middleware
- Lifecycle nodes provide a standard interface for node state management