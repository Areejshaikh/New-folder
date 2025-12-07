#!/usr/bin/env python3

"""
Python Manipulation Agent Example.

This node demonstrates how to create a Python agent that performs manipulation tasks.
The agent uses joint control to move a robotic arm to perform simple pick-and-place operations.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import time


class PythonManipulationAgent(Node):
    """A Python agent for robotic arm manipulation tasks."""

    def __init__(self):
        """Initialize the node and set up publishers/subscribers."""
        super().__init__('python_manipulation_agent')
        
        # Publisher for joint trajectory commands
        self.joint_publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Subscriber for joint states
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscriber for manipulation commands
        self.command_subscriber = self.create_subscription(
            String,
            '/manipulation_command',
            self.command_callback,
            10
        )
        
        # Agent state variables
        self.current_joint_positions = {}
        self.joint_names = []  # Will be populated from joint states
        self.manipulation_active = False
        
        # Predefined joint positions for different arm poses
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [joint1, joint2, joint3, joint4, joint5, joint6]
        self.pick_position = [0.5, 0.7, -1.0, 0.3, 0.2, 0.0]  # Adjusted for picking
        self.place_position = [-0.5, 0.7, -1.0, 0.3, 0.2, 0.0]  # Adjusted for placing

    def joint_state_callback(self, msg):
        """
        Callback function for joint state updates.
        
        Args:
            msg: JointState message containing current joint positions
        """
        self.joint_names = list(msg.name)
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def command_callback(self, msg):
        """
        Callback function for manipulation commands.
        
        Args:
            msg: String message containing the manipulation command
        """
        command = msg.data.lower()
        
        if command == 'home':
            self.move_to_position(self.home_position, duration=3.0)
        elif command == 'pick':
            self.move_to_position(self.pick_position, duration=3.0)
        elif command == 'place':
            self.move_to_position(self.place_position, duration=3.0)
        elif command == 'wave':
            self.wave_action()
        else:
            self.get_logger().info(f'Unknown manipulation command: {command}')

    def move_to_position(self, positions, duration=3.0):
        """
        Move the arm to a specified joint position.
        
        Args:
            positions: List of joint positions to move to
            duration: Time in seconds to complete the movement
        """
        if not self.joint_names:
            self.get_logger().warn('Joint names not yet received, cannot move arm')
            return

        # Ensure we have the right number of positions
        if len(positions) != len(self.joint_names):
            self.get_logger().warn(f'Position count mismatch: {len(positions)} positions for {len(self.joint_names)} joints')
            return

        # Create joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory_msg.points.append(point)
        
        # Publish the trajectory
        self.joint_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Moving arm to position: {positions}')

    def wave_action(self):
        """Perform a wave action with the arm."""
        self.get_logger().info('Performing wave action')
        
        # Current position
        current_pos = list(self.current_joint_positions.values()) if self.current_joint_positions else self.home_position
        
        # Wave positions (simplified for 3 joints)
        wave_positions = [
            [0.0, 0.5, 0.0, current_pos[3] if len(current_pos) > 3 else 0.0, current_pos[4] if len(current_pos) > 4 else 0.0, current_pos[5] if len(current_pos) > 5 else 0.0],
            [0.3, 0.5, 0.0, current_pos[3] if len(current_pos) > 3 else 0.0, current_pos[4] if len(current_pos) > 4 else 0.0, current_pos[5] if len(current_pos) > 5 else 0.0],
            [-0.3, 0.5, 0.0, current_pos[3] if len(current_pos) > 3 else 0.0, current_pos[4] if len(current_pos) > 4 else 0.0, current_pos[5] if len(current_pos) > 5 else 0.0],
            [0.0, 0.5, 0.0, current_pos[3] if len(current_pos) > 3 else 0.0, current_pos[4] if len(current_pos) > 4 else 0.0, current_pos[5] if len(current_pos) > 5 else 0.0]
        ]
        
        for i, pos in enumerate(wave_positions):
            self.get_logger().info(f'Wave step {i+1}/{len(wave_positions)}')
            self.move_to_position(pos, duration=1.0)
            time.sleep(1.2)  # Wait for movement to complete (plus a little extra)


def main(args=None):
    """Main function."""
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the manipulation agent
    agent = PythonManipulationAgent()
    
    # Log a startup message
    agent.get_logger().info('Python Manipulation Agent started')
    agent.get_logger().info('Send commands via /manipulation_command topic: home, pick, place, wave')
    
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