#!/usr/bin/env python3

"""
Python Navigation Agent Example.

This node demonstrates how to create a Python agent that performs navigation tasks.
The agent uses sensor data to navigate through an environment while avoiding obstacles.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class PythonNavigationAgent(Node):
    """A Python agent for robot navigation with obstacle avoidance."""

    def __init__(self):
        """Initialize the node and set up publishers/subscribers."""
        super().__init__('python_navigation_agent')
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Create subscriber for navigation commands (start/stop)
        self.nav_command_subscriber = self.create_subscription(
            Bool,
            '/nav_command',
            self.nav_command_callback,
            10
        )
        
        # Create a timer for the control loop (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Agent state variables
        self.laser_data = None
        self.navigation_active = True  # Start with navigation enabled
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.obstacle_detected = False
        self.safe_distance = 0.8  # meters
        self.mode = 'explore'  # 'explore' or 'avoid'

    def scan_callback(self, msg):
        """
        Callback function for processing laser scan data.
        
        Args:
            msg: LaserScan message containing distance measurements
        """
        self.laser_data = msg
        
        # Check for obstacles in front of the robot
        if len(msg.ranges) > 0:
            # Only consider valid (non-infinite) range values in the front 90-degree sector
            front_ranges_idx = len(msg.ranges) // 2  # Middle index
            front_sector_size = len(msg.ranges) // 4  # 90-degree sector
            
            start_idx = max(0, front_ranges_idx - front_sector_size // 2)
            end_idx = min(len(msg.ranges), front_ranges_idx + front_sector_size // 2)
            
            front_ranges = msg.ranges[start_idx:end_idx]
            valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
            
            if valid_ranges:
                min_front_distance = min(valid_ranges)
                self.obstacle_detected = min_front_distance < self.safe_distance
                self.get_logger().debug(f'Min front distance: {min_front_distance:.2f}m')

    def nav_command_callback(self, msg):
        """
        Callback function for navigation commands (start/stop).
        
        Args:
            msg: Bool message containing the navigation command (True to start)
        """
        self.navigation_active = msg.data
        status = "enabled" if self.navigation_active else "disabled"
        self.get_logger().info(f'Navigation {status}')

    def control_loop(self):
        """
        Main control loop that runs at 10Hz.
        This function determines the robot's navigation behavior based on sensor data.
        """
        if not self.navigation_active:
            # Navigation disabled, stop the robot
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        elif self.laser_data is None:
            # No sensor data yet, stop the robot
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        elif self.obstacle_detected:
            # Obstacle detected - switch to obstacle avoidance mode
            self.mode = 'avoid'
            self.linear_velocity = 0.0
            self.angular_velocity = 0.8  # Turn right to avoid obstacle
        else:
            # Path is clear - explore mode
            if self.mode == 'avoid':
                # Just cleared an obstacle, turn slightly to get back on track
                self.linear_velocity = 0.2
                self.angular_velocity = 0.2
                # After a short time, return to straight movement
                self.get_logger().debug('Returning to exploration after obstacle')
            else:
                # Normal exploration - move forward
                self.linear_velocity = 0.4  # Move forward at 0.4 m/s
                self.angular_velocity = 0.0

        # Create and publish the velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_velocity
        cmd_msg.angular.z = self.angular_velocity
        
        self.cmd_vel_publisher.publish(cmd_msg)
        
        # Log the command and state
        self.get_logger().info(
            f'Nav {self.mode}: lin={cmd_msg.linear.x:.2f}, ang={cmd_msg.angular.z:.2f}'
        )


def main(args=None):
    """Main function."""
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the navigation agent
    agent = PythonNavigationAgent()
    
    # Log a startup message
    agent.get_logger().info('Python Navigation Agent started')
    agent.get_logger().info('Use /nav_command topic to enable/disable navigation')
    
    try:
        # Keep the node running until it's shut down
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted by user')
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        agent.cmd_vel_publisher.publish(stop_msg)
        
        # Clean up when done
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()