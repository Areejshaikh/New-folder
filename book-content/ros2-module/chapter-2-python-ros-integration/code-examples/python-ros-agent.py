#!/usr/bin/env python3

"""
Python ROS Agent Example.

This node demonstrates how to create a Python agent that interfaces with ROS 2 using rclpy.
The agent subscribes to sensor data, processes it, and publishes commands to control a robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class PythonROSControlAgent(Node):
    """A Python agent that controls a robot based on sensor data."""

    def __init__(self):
        """Initialize the node and set up publishers/subscribers."""
        super().__init__('python_ros_agent')
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Create a timer for the control loop (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Agent state variables
        self.laser_data = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.obstacle_detected = False
        self.safe_distance = 0.5  # meters

    def scan_callback(self, msg):
        """
        Callback function for processing laser scan data.
        
        Args:
            msg: LaserScan message containing distance measurements
        """
        self.laser_data = msg
        # Check for obstacles in front of the robot
        # Use the middle range value as a simple approximation
        if len(msg.ranges) > 0:
            # Only consider valid (non-infinite) range values
            valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                min_distance = min(valid_ranges)
                self.obstacle_detected = min_distance < self.safe_distance
                self.get_logger().debug(f'Min distance: {min_distance:.2f}m, Obstacle: {self.obstacle_detected}')

    def control_loop(self):
        """
        Main control loop that runs at 10Hz.
        This function determines the robot's movement based on sensor data.
        """
        if self.laser_data is None:
            # No sensor data yet, stop the robot
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        elif self.obstacle_detected:
            # Obstacle detected - stop and turn
            self.linear_velocity = 0.0
            self.angular_velocity = 0.5  # Turn right
        else:
            # Path is clear - move forward
            self.linear_velocity = 0.3  # Move forward at 0.3 m/s
            self.angular_velocity = 0.0

        # Create and publish the velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_velocity
        cmd_msg.angular.z = self.angular_velocity
        
        self.cmd_vel_publisher.publish(cmd_msg)
        
        # Log the command
        self.get_logger().info(
            f'Vel cmd: linear={cmd_msg.linear.x:.2f}, angular={cmd_msg.angular.z:.2f}'
        )


def main(args=None):
    """Main function."""
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the Python ROS agent
    agent = PythonROSControlAgent()
    
    # Log a startup message
    agent.get_logger().info('Python ROS Control Agent started')
    
    try:
        # Keep the node running until it's shut down
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted by user')
    finally:
        # Clean up when done
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()