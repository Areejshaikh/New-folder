# Chapter 2: Bridging Python Agents with ROS 2

## Overview
This chapter explores how to connect Python agents with ROS 2 using the rclpy library. We'll cover the fundamentals of the Python-ROS interface, create practical examples of Python agents that interact with ROS 2 systems, and demonstrate how to implement navigation and manipulation tasks.

## Learning Objectives
By the end of this chapter, you will:
- Understand how to use rclpy to connect Python agents to ROS 2
- Be able to implement Python agents that can control robots
- Know how to create navigation and manipulation examples
- Be familiar with best practices for Python-ROS integration

## Table of Contents
1. [Introduction to rclpy](#introduction-to-rclpy)
2. [Setting up Python-ROS Connection](#setting-up-python-ros-connection)
3. [Python Agent Structure](#python-agent-structure)
4. [Navigation Example](#navigation-example)
5. [Manipulation Example](#manipulation-example)
6. [Troubleshooting](#troubleshooting)
7. [Summary](#summary)

## Introduction to rclpy
rclpy is the Python client library for ROS 2. It provides the standard interface for Python programs to interface with ROS 2. rclpy provides services such as:
- Creating nodes
- Publishing and subscribing to topics
- Making service calls
- Creating service servers
- Creating action clients and servers

rclpy is the Python equivalent of rclcpp (the C++ client library) and provides the same functionality as the underlying rcl (ROS Client Library) implementation.

## Setting up Python-ROS Connection
To connect Python with ROS 2, you need to:
1. Initialize the ROS client library
2. Create a node
3. Create publishers, subscribers, services, or clients
4. Spin the node to process callbacks

Here's a basic example of initializing a Python ROS node:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of your node class
    my_node = MyPythonNode()

    # Start processing callbacks
    rclpy.spin(my_node)

    # Clean up when done
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python Agent Structure
A Python agent that interfaces with ROS 2 typically follows this structure:

```python
import rclpy
from rclpy.node import Node
# Import any custom message types you need

class PythonROSAgent(Node):
    def __init__(self):
        # Initialize the parent Node class
        super().__init__('python_ros_agent')

        # Create publishers, subscribers, services, etc.
        self.publisher = self.create_publisher(MessageType, 'topic_name', 10)
        self.subscriber = self.create_subscription(MessageType, 'topic_name', self.callback, 10)

        # Any agent-specific initialization
        self.agent_state = {}

    def callback(self, msg):
        # Process incoming messages
        pass

def main(args=None):
    rclpy.init(args=args)
    agent = PythonROSAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Navigation Example
This section demonstrates how to create a Python agent that performs simple navigation tasks. The navigation example (python-navigation-agent.py) shows how a Python agent can process sensor data and make navigation decisions:

```python
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
```

This navigation agent demonstrates several key concepts:
- Subscribing to sensor data (LaserScan messages)
- Processing sensor data to detect obstacles
- Making navigation decisions based on sensor inputs
- Publishing velocity commands to control the robot
- Using state management to track navigation modes

## Manipulation Example
This section demonstrates how to create a Python agent that performs simple manipulation tasks. The manipulation example (python-manipulation-agent.py) shows how to control a robotic arm:

```python
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
```

This manipulation agent demonstrates several key concepts:
- Controlling a robotic arm using joint trajectories
- Subscribing to joint state feedback
- Accepting commands through a dedicated topic
- Predefining joint positions for different tasks
- Implementing multi-step actions like waving

## Step-by-Step Tutorial for Implementing Python Agents
Here's a step-by-step guide to implementing your own Python agent:

### Step 1: Define Your Agent Class
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyPythonAgent(Node):
    def __init__(self):
        super().__init__('my_python_agent')
        # Initialize publishers, subscribers, and timers here
```

### Step 2: Create Publishers and Subscribers
```python
def __init__(self):
    super().__init__('my_python_agent')

    # Create publisher
    self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    # Create subscriber
    self.subscriber = self.create_subscription(
        String,  # Message type
        '/sensor_data',  # Topic name
        self.sensor_callback,  # Callback function
        10  # Queue size
    )
```

### Step 3: Implement Callback Functions
```python
def sensor_callback(self, msg):
    # Process the incoming message
    self.get_logger().info(f'Received: {msg.data}')
    # Implement your agent logic here
```

### Step 4: Add Timers for Periodic Actions
```python
def __init__(self):
    super().__init__('my_python_agent')
    # ... other initialization ...

    # Create a timer to execute actions periodically (e.g., at 10 Hz)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.control_loop)

def control_loop(self):
    # Implement periodic control logic here
    cmd = Twist()
    # Set velocity based on agent state
    self.publisher.publish(cmd)
```

### Step 5: Implement Main Function
```python
def main(args=None):
    rclpy.init(args=args)
    agent = MyPythonAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises for Practicing Python Agent Development
1. Modify the navigation agent to implement a different obstacle avoidance strategy (e.g., wall following)
2. Create a Python agent that follows a person detected by a camera
3. Implement a patrol agent that visits predefined waypoints
4. Create a Python agent that responds to voice commands
5. Develop a Python agent that coordinates with other agents to perform a task
6. Extend the manipulation agent to handle multiple objects
7. Create a Python agent that learns from sensor data to improve its behavior
8. Implement a multi-agent system for cooperative manipulation tasks

## Troubleshooting
Common issues when connecting Python agents with ROS 2:

1. **Import errors**: Ensure rclpy is installed (`pip install rclpy` or it's available in your ROS 2 environment)
2. **Node name conflicts**: Each node should have a unique name within the ROS graph
3. **Topic/service not found**: Verify that the intended topic/service exists and is available
4. **Connection timeouts**: Check that all required nodes are running and the ROS network is properly configured

### Common Solutions:
- Source your ROS 2 installation: `source /opt/ros/humble/setup.bash`
- Check available topics: `ros2 topic list`
- Check available services: `ros2 service list`
- Verify message types: `ros2 interface show <msg_type>`

## Summary
This chapter covered how to bridge Python agents with ROS 2 using the rclpy library. You learned about the structure of Python agents, implemented navigation and manipulation examples, and learned how to troubleshoot common issues. Through the examples, you saw how Python agents can process sensor data, make decisions, and control robotic systems. In the next chapter, we'll explore creating robot models using URDF (Unified Robot Description Format).

---

## Advanced Notes (Optional)
- Use composition to run multiple nodes in the same process for better performance
- Consider using async/await patterns for non-blocking operations in Python agents
- Parameter management can help configure agents dynamically
- Use actions for long-running tasks that provide feedback