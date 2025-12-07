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