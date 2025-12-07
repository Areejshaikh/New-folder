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