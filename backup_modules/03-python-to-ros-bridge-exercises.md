---
title: "Hands-On Exercises: Bridging Python Agents to ROS 2 Controllers"
sidebar_label: "Python to ROS Bridge Exercises"
description: "Practical exercises for connecting Python agents with ROS 2 controllers using rclpy"
---

# Hands-On Exercises: Bridging Python Agents to ROS 2 Controllers

## Overview

This module contains hands-on exercises that will teach you how to bridge Python agents (such as AI algorithms or decision-making systems) with ROS 2 controllers. These exercises build on the concepts from previous modules and provide practical experience implementing the communication patterns that connect digital intelligence to physical robotic systems.

## Prerequisites

Before starting these exercises, you should have:

- Completed Modules 1 and 2 (ROS 2 Introduction and Architecture)
- Basic Python programming experience
- Understanding of ROS 2 concepts (nodes, topics, services, actions)
- Working ROS 2 environment with Python development tools

## Learning Objectives

By completing these exercises, you will be able to:

1. Create nodes that bridge Python agents with ROS 2 controllers
2. Design publishers and subscribers for AI-robot communication
3. Implement service clients and servers for AI-powered robot control
4. Build action clients for complex robot behaviors orchestrated by AI
5. Structure Python code using rclpy for optimal performance and maintainability

## Exercise 1: Simple AI Decision Node

### Objective
Create a simple AI node that makes decisions and communicates with a ROS 2 controller.

### Instructions
1. Create a new package called `ai_bridge_examples`
2. Create a node that simulates an AI agent making navigation decisions
3. Have the AI node publish velocity commands to control a simulated robot
4. Implement a subscriber to receive sensor data from the robot

### Requirements
- The AI node must use a timer to make decisions at regular intervals
- The node should subscribe to a `/robot/laser_scan` topic to receive sensor data
- The node should publish to `/robot/cmd_vel` topic to control movement
- The AI algorithm should make decisions based on sensor data (avoid obstacles)
- Use appropriate QoS settings for sensor data vs control commands

### Solution Template
```python
# ai_bridge_examples/ai_bridge_examples/simple_ai_agent.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math


class SimpleAIAgent(Node):
    """
    Simple AI agent that makes navigation decisions based on sensor data
    """
    
    def __init__(self):
        super().__init__('simple_ai_agent')
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        # Create subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/robot/laser_scan',
            self.scan_callback,
            10
        )
        
        # Create subscriber for distance to obstacle
        self.obstacle_distance_publisher = self.create_publisher(Float32, '/obstacle_distance', 10)
        
        # Timer to control decision-making frequency
        self.timer = self.create_timer(0.1, self.decision_callback)  # 10 Hz
        
        # Track the closest obstacle distance
        self.closest_obstacle_distance = float('inf')
        
        self.get_logger().info('Simple AI Agent initialized')
    
    def scan_callback(self, msg):
        """
        Process laser scan data to find the closest obstacle
        """
        # Find minimum range in the scan (ignoring invalid measurements)
        valid_ranges = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r))]
        if valid_ranges:
            self.closest_obstacle_distance = min(valid_ranges)
            self.get_logger().debug(f'Closest obstacle: {self.closest_obstacle_distance:.2f}m')
    
    def decision_callback(self):
        """
        Make navigation decisions based on sensor data
        """
        # Create a velocity command message
        cmd_vel = Twist()
        
        # If obstacle is close, turn to avoid it
        if self.closest_obstacle_distance < 1.0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
        else:
            # Safe to move forward
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
        
        # Publish the command
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Publish obstacle distance for monitoring
        distance_msg = Float32()
        distance_msg.data = self.closest_obstacle_distance
        self.obstacle_distance_publisher.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    
    ai_agent = SimpleAIAgent()
    
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('Shutting down AI agent node')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise Steps
1. Create this node in your workspace
2. Build the package with `colcon build`
3. Source the setup files
4. Run the node to test the functionality
5. Verify that commands are being published to `/robot/cmd_vel`
6. Monitor the `/obstacle_distance` topic to ensure obstacle detection is working

### Expected Outcome
The AI agent should detect obstacles in its path and adjust its movement to avoid them, publishing appropriate velocity commands to the ROS 2 controller.

---

## Exercise 2: Service-Based AI Decision Making

### Objective
Implement an AI service that receives complex requests from other nodes and returns calculated responses.

### Instructions
1. Create an AI service server that computes optimal navigation paths
2. Implement a service client that requests path planning from the AI service
3. Use custom service messages for the path planning interface

### Creating a Custom Service Message
First, create a custom service definition in `ai_bridge_examples/srv/ComputePath.srv`:

```
# Request
float64 start_x
float64 start_y
float64 start_theta
float64 goal_x
float64 goal_y
string[] constraints  # Robot constraints or forbidden areas

---
# Response
bool success
float64[] path_x  # Array of x coordinates in the path
float64[] path_y  # Array of y coordinates in the path
float64[] path_theta  # Array of orientations in the path
float64 execution_time
string message
```

### Service Server Implementation
```python
# ai_bridge_examples/ai_bridge_examples/path_planning_service.py

import rclpy
from rclpy.node import Node
from ai_bridge_examples.srv import ComputePath  # This would be your custom service
import math
from collections import deque
import numpy as np


class PathPlanningService(Node):
    """
    AI-powered service that computes optimal paths for robot navigation
    """
    
    def __init__(self):
        super().__init__('path_planning_service')
        
        # Create the service
        self.srv = self.create_service(
            ComputePath,
            'compute_navigation_path',
            self.compute_path_callback
        )
        
        self.get_logger().info('Path Planning Service initialized')
    
    def compute_path_callback(self, request, response):
        """
        Compute an optimal path from start to goal position
        """
        self.get_logger().info(f'Received path planning request from ({request.start_x}, {request.start_y}) to ({request.goal_x}, {request.goal_y})')
        
        try:
            # Simplified path planning algorithm (in reality, this would use a complex planner)
            path_x, path_y, path_theta = self.simple_planner(
                request.start_x, request.start_y, request.start_theta,
                request.goal_x, request.goal_y,
                request.constraints
            )
            
            # Fill response
            response.success = True
            response.path_x = path_x
            response.path_y = path_y
            response.path_theta = path_theta
            response.execution_time = 0.1  # Placeholder execution time
            response.message = "Path computed successfully"
            
            self.get_logger().info(f'Returned path with {len(path_x)} waypoints')
            
        except Exception as e:
            response.success = False
            response.message = f"Path planning failed: {str(e)}"
            self.get_logger().error(f'Path planning error: {str(e)}')
        
        return response
    
    def simple_planner(self, start_x, start_y, start_theta, goal_x, goal_y, constraints):
        """
        Simplified path planning algorithm
        In a real implementation, this would use A*, Dijkstra, RRT*, or other algorithms
        """
        # For simplicity, create a straight-line path
        # In practice, this would implement a proper path planning algorithm
        steps = 20  # Number of waypoints
        path_x = []
        path_y = []
        path_theta = []
        
        dx = (goal_x - start_x) / steps
        dy = (goal_y - start_y) / steps
        dtheta = (0 - start_theta) / steps  # Simplified rotation calculation
        
        for i in range(steps + 1):
            path_x.append(start_x + dx * i)
            path_y.append(start_y + dy * i)
            path_theta.append(start_theta + dtheta * i)
        
        return path_x, path_y, path_theta


def main(args=None):
    rclpy.init(args=args)
    
    path_planner_service = PathPlanningService()
    
    try:
        rclpy.spin(path_planner_service)
    except KeyboardInterrupt:
        path_planner_service.get_logger().info('Shutting down path planning service')
    finally:
        path_planner_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client Implementation
```python
# ai_bridge_examples/ai_bridge_examples/path_planning_client.py

import rclpy
from rclpy.node import Node
from ai_bridge_examples.srv import ComputePath  # This would be your custom service
import sys


class PathPlanningClient(Node):
    """
    Client that requests path planning from the AI service
    """
    
    def __init__(self):
        super().__init__('path_planning_client')
        
        # Create the client
        self.client = self.create_client(ComputePath, 'compute_navigation_path')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Path planning service not available, waiting again...')
        
        self.request = ComputePath.Request()
        
        # Create a timer to periodically request new paths
        self.timer = self.create_timer(2.0, self.send_request)
        self.count = 0
    
    def send_request(self):
        """
        Send a path planning request to the service
        """
        # In this example, we'll vary the goal position each time
        self.request.start_x = 0.0
        self.request.start_y = 0.0
        self.request.start_theta = 0.0
        self.request.goal_x = 5.0 + self.count * 2.0  # Vary goal position
        self.request.goal_y = 5.0 + self.count * 1.5
        self.request.constraints = []  # No constraints in this example
        
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)
        
        self.get_logger().info(f'Sent path planning request #{self.count}')
        self.count += 1
    
    def response_callback(self, future):
        """
        Process the response from the path planning service
        """
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'Path computed with {len(response.path_x)} waypoints')
                # In a real implementation, you might send these waypoints to a navigation node
            else:
                self.get_logger().error(f'Path planning failed: {response.message}')
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    client = PathPlanningClient()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Shutting down path planning client')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise Steps
1. Define your custom service message in the appropriate directory
2. Implement both the service server and client nodes
3. Test the communication between them
4. Verify that path requests are processed correctly
5. Modify constraints to see how the AI service adapts to changing conditions

### Expected Outcome
The AI service should receive path planning requests from clients, compute appropriate paths considering any constraints, and return the path to the client for execution by a navigation system.

---

## Exercise 3: Action-Based Task Orchestration

### Objective
Create an AI action client that orchestrates complex robot behaviors using ROS 2 actions.

### Instructions
1. Develop an AI action client that sends tasks to a robot's manipulation action server
2. Handle feedback and result messages appropriately
3. Implement cancellation capabilities for AI-driven task interruption

### Example Action Client
```python
# ai_bridge_examples/ai_bridge_examples/manipulation_ai_client.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory  # Standard action
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ManipulationAIClient(Node):
    """
    AI node that orchestrates complex robot manipulator behaviors
    """
    
    def __init__(self):
        super().__init__('manipulation_ai_client')
        
        # Create action client to send trajectories to the manipulator controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Create timer to send tasks periodically
        self.timer = self.create_timer(5.0, self.send_goal)
        
        self.goal_sent = False
        
        self.get_logger().info('Manipulation AI Client initialized')
    
    def send_goal(self):
        """
        Send a manipulation goal to the robot
        """
        if self.goal_sent:
            # Don't send another goal until the current one is done
            return
        
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return
        
        # Create a simple trajectory goal (in a real application, AI would calculate this)
        goal_msg = FollowJointTrajectory.Goal()
        
        # Define joint names (these would correspond to your robot's manipulator joints)
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal_msg.trajectory.joint_names = joint_names
        
        # Create trajectory points
        trajectory_points = []
        
        # Point 1: Initial position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=1, nanosec=0)
        trajectory_points.append(point1)
        
        # Point 2: Grasp position
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.2, -0.3, 0.1, 0.4, -0.2]
        point2.time_from_start = Duration(sec=3, nanosec=0)
        trajectory_points.append(point2)
        
        # Point 3: Lift position
        point3 = JointTrajectoryPoint()
        point3.positions = [0.5, 0.5, -0.3, 0.1, 0.4, -0.2]
        point3.time_from_start = Duration(sec=5, nanosec=0)
        trajectory_points.append(point3)
        
        goal_msg.trajectory.points = trajectory_points
        
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info('Sent manipulation goal to robot')
        self.goal_sent = True
    
    def goal_response_callback(self, future):
        """
        Handle the response from the action server about accepting the goal
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            self.goal_sent = False
            return
        
        self.get_logger().info('Goal accepted by server')
        
        # Asynchronously get the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server during goal execution
        """
        feedback = feedback_msg.feedback
        # In a real AI system, this feedback would be analyzed and possibly influence 
        # the next actions the AI decides to take
        self.get_logger().debug(f'Received feedback: Current joint positions: {feedback.joint_names}')
    
    def get_result_callback(self, future):
        """
        Handle the final result of the action
        """
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result.error_code}')
        
        # Reset flag so we can send another goal
        self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)
    
    action_client = ManipulationAIClient()
    
    # Use a multi-threaded executor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_client.get_logger().info('Shutting down manipulation AI client')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise Steps
1. Implement the action client with appropriate AI decision-making logic
2. Set up feedback handling for dynamic adjustments
3. Test with a trajectory controller (or simulated controller)
4. Implement cancellation logic to interrupt tasks when AI decides conditions warrant it
5. Add logic to chain multiple actions together based on AI reasoning

### Expected Outcome
The AI action client should be able to orchestrate complex manipulator behaviors by sending action goals, receiving real-time feedback, and adjusting its plans based on the robot's progress.

---

## Exercise 4: AI-Driven Behavior Tree Implementation

### Objective
Implement a simple behavior tree system in Python that interfaces with ROS 2 for robot control.

### Instructions
1. Create nodes for common robotics behaviors (move_to, grasp_object, detect_person, etc.)
2. Implement the behavior tree logic in Python
3. Connect each behavior node to ROS 2 services or actions
4. Create a main AI node that runs the behavior tree based on environmental stimuli

### Basic Behavior Tree Framework
```python
# ai_bridge_examples/ai_bridge_examples/behavior_tree.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from enum import Enum


class NodeStatus(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class BehaviorNode:
    """
    Base class for all behavior tree nodes
    """
    def __init__(self, name):
        self.name = name
    
    def tick(self):
        """
        Execute one cycle of the behavior
        """
        raise NotImplementedError


class SelectorNode(BehaviorNode):
    """
    Selector node (runs children until one succeeds)
    """
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_child_idx = 0
    
    def tick(self):
        for i, child in enumerate(self.children):
            status = child.tick()
            if status == NodeStatus.SUCCESS:
                self.current_child_idx = 0  # Reset on success
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                self.current_child_idx = i
                return NodeStatus.RUNNING
        
        # All children failed
        self.current_child_idx = 0
        return NodeStatus.FAILURE


class SequenceNode(BehaviorNode):
    """
    Sequence node (runs all children in sequence until one fails)
    """
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_child_idx = 0
    
    def tick(self):
        for i in range(self.current_child_idx, len(self.children)):
            child = self.children[i]
            status = child.tick()
            if status == NodeStatus.FAILURE:
                self.current_child_idx = 0  # Reset on failure
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                self.current_child_idx = i
                return NodeStatus.RUNNING
            # If SUCCESS, continue to next child
            self.current_child_idx = i + 1
        
        # All children succeeded
        self.current_child_idx = 0
        return NodeStatus.SUCCESS


class PatrolBehavior(Node):
    """
    Node that implements patrol behavior using behavior tree
    """
    def __init__(self):
        super().__init__('patrol_behavior')
        
        # Publishers for movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers for sensor data
        self.obstacle_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # For this exercise, we'll run the BT at 10 Hz
        self.timer = self.create_timer(0.1, self.bt_tick)
        
        # Create behavior tree
        self.setup_behavior_tree()
        
        # Track state
        self.obstacle_detected = False
        
        self.get_logger().info('Patrol behavior node initialized')
    
    def setup_behavior_tree(self):
        """
        Set up the behavior tree structure
        """
        # Leaf nodes (atomic behaviors)
        self.move_forward_node = MoveForwardNode(self)
        self.turn_around_node = TurnAroundNode(self)
        self.avoid_obstacle_node = AvoidObstacleNode(self)
        
        # Selector: try to move forward, if obstructed, handle obstacle
        self.navigation_selector = SelectorNode(
            "navigation",
            [self.move_forward_node, self.avoid_obstacle_node]
        )
        
        # Root of the tree
        self.root = self.navigation_selector
    
    def scan_callback(self, msg):
        """
        Update obstacle detection status
        """
        # Check if any distances in the scan are below threshold
        if len(msg.ranges) > 0:
            min_distance = min([r for r in msg.ranges if not (r != r)])  # Check for NaN
            if not (min_distance != min_distance):  # Check for NaN
                self.obstacle_detected = min_distance < 0.5  # 0.5m threshold
    
    def bt_tick(self):
        """
        Tick the behavior tree
        """
        self.root.tick()


class MoveForwardNode(BehaviorNode):
    """
    Leaf node for moving forward
    """
    def __init__(self, patrol_node):
        super().__init__("move_forward")
        self.patrol_node = patrol_node
        self.movement_time = 0.0
        self.target_time = 5.0  # Move forward for 5 seconds
    
    def tick(self):
        if self.patrol_node.obstacle_detected:
            self.movement_time = 0.0
            return NodeStatus.FAILURE
        
        # Publish forward command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
        
        self.movement_time += 0.1  # Increment by timer period
        
        if self.movement_time >= self.target_time:
            cmd_vel.linear.x = 0.0
            self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
            self.movement_time = 0.0
            return NodeStatus.SUCCESS
        
        return NodeStatus.RUNNING


class TurnAroundNode(BehaviorNode):
    """
    Leaf node for turning around
    """
    def __init__(self, patrol_node):
        super().__init__("turn_around")
        self.patrol_node = patrol_node
        self.rotation_time = 0.0
        self.target_time = 3.0  # Rotate for 3 seconds
    
    def tick(self):
        # Publish rotation command
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.5
        self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
        
        self.rotation_time += 0.1  # Increment by timer period
        
        if self.rotation_time >= self.target_time:
            cmd_vel.angular.z = 0.0
            self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
            self.rotation_time = 0.0
            return NodeStatus.SUCCESS
        
        return NodeStatus.RUNNING


class AvoidObstacleNode(BehaviorNode):
    """
    Leaf node for avoiding obstacles
    """
    def __init__(self, patrol_node):
        super().__init__("avoid_obstacle")
        self.patrol_node = patrol_node
        self.avoid_time = 0.0
        self.target_time = 2.0  # Avoid for 2 seconds
    
    def tick(self):
        if not self.patrol_node.obstacle_detected:
            self.avoid_time = 0.0
            return NodeStatus.FAILURE
        
        # Publish avoidance command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.5  # Turn right to avoid
        self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
        
        self.avoid_time += 0.1  # Increment by timer period
        
        if self.avoid_time >= self.target_time:
            cmd_vel.angular.z = 0.0
            self.patrol_node.cmd_vel_publisher.publish(cmd_vel)
            self.avoid_time = 0.0
            return NodeStatus.SUCCESS
        
        return NodeStatus.RUNNING


def main(args=None):
    rclpy.init(args=args)
    
    patrol_node = PatrolBehavior()
    
    try:
        rclpy.spin(patrol_node)
    except KeyboardInterrupt:
        patrol_node.get_logger().info('Shutting down patrol behavior node')
    finally:
        patrol_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise Steps
1. Implement the behavior tree framework with selector and sequence nodes
2. Create leaf behavior nodes for common robotics actions
3. Connect the Python behavior tree with ROS 2 topics and services
4. Test the patrol behavior with simulated robot and sensor data
5. Extend the tree with additional behaviors (investigate_sound, pick_up_object, etc.)

### Expected Outcome
The behavior tree should implement a patrol pattern that responds to obstacles and other environmental stimuli, demonstrating how AI decision-making can be structured in a ROS 2 robot system.

---

## Exercise 5: Complete AI-Robot Bridge Application

### Objective
Combine all learned concepts into a single application that interfaces AI decision-making with physical robot control.

### Instructions
1. Create a modular application connecting AI processing to robot control
2. Implement state management between AI decisions and robot actions
3. Add error handling and recovery mechanisms
4. Demonstrate all communication patterns (topics, services, actions)
5. Include logging and diagnostics for debugging

### Complete Example Application
```python
# ai_bridge_examples/ai_bridge_examples/integrated_ai_robot_bridge.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

# Import messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from example_interfaces.srv import Trigger

# Import behavior tree components
from .behavior_tree import *
# Import service components
from .path_planning_service import PathPlanningService
# Import AI agent components
from .simple_ai_agent import SimpleAIAgent


class IntegratedAIBridge(Node):
    """
    Complete application combining AI processing with robot control
    """
    
    def __init__(self):
        super().__init__('integrated_ai_bridge')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/ai_bridge/status', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Services
        self.emergency_stop_service = self.create_service(
            SetBool,
            'emergency_stop',
            self.emergency_stop_callback
        )
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status tracking
        self.robot_operational = True
        self.scan_data = None
        self.last_command_time = self.get_clock().now()
        
        # Initialize AI components
        self.ai_agent = SimpleAIAgent(self)  # Pass node reference to AI agent
        self.path_planner = PathPlanningService(self.get_namespace())  # Separate node for path planning
        
        self.get_logger().info('Integrated AI-Robot Bridge initialized')
    
    def scan_callback(self, msg):
        """
        Update sensor data for AI processing
        """
        self.scan_data = msg
        
        # Publish status for monitoring
        status_msg = String()
        status_msg.data = f"Scan received with {len(msg.ranges)} ranges"
        self.status_pub.publish(status_msg)
    
    def emergency_stop_callback(self, request, response):
        """
        Handle emergency stop requests
        """
        if request.data:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            # Send stop command to robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.robot_operational = False
        else:
            self.get_logger().info('Emergency stop cleared')
            self.robot_operational = True
        
        response.success = True
        response.message = f"Emergency stop {(not request.data and 'deactivated' or 'activated')}"
        return response
    
    def control_loop(self):
        """
        Main control loop that makes AI decisions based on sensor data
        """
        if not self.robot_operational:
            return
        
        if self.scan_data is not None:
            # Process sensor data with AI components
            cmd_vel = self.ai_agent.process_sensor_data(self.scan_data)
            
            # Publish the command
            self.cmd_vel_pub.publish(cmd_vel)
            self.last_command_time = self.get_clock().now()
    
    def destroy_node(self):
        """
        Clean up when shutting down
        """
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        super().destroy_node()


# Helper class to process sensor data with AI
class SimpleAIDataProcessor:
    """
    Class to encapsulate AI processing logic
    """
    
    def __init__(self, node):
        self.node = node
    
    def process_sensor_data(self, scan_data):
        """
        Process sensor data and return appropriate command
        """
        # Find closest obstacle
        valid_ranges = [r for r in scan_data.ranges if r > 0 and r < float('inf')]
        
        if not valid_ranges:
            # No valid measurements, go forward
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            return cmd_vel
        
        min_range = min(valid_ranges)
        
        # Simple reactive control
        cmd_vel = Twist()
        if min_range < 0.5:  # Obstacle closer than 50cm
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3  # Turn away from obstacle
        else:
            cmd_vel.linear.x = 0.3  # Move forward
            cmd_vel.angular.z = 0.0
        
        return cmd_vel


def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes
    ai_bridge = IntegratedAIBridge()
    
    # Use multi-threaded executor to handle all callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(ai_bridge)
    executor.add_node(ai_bridge.path_planner)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('Shutting down integrated AI-robot bridge')
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise Steps
1. Integrate all previously created components into a cohesive system
2. Test the system with simulated or real robot data
3. Adjust parameters to optimize performance
4. Add error handling for network disruptions or sensor failures
5. Verify all three communication methods work in concert

### Expected Outcome
The integrated application should demonstrate how an AI system can process sensor data and control a robot using all the communication patterns studied in this module.

## Summary

In these exercises, you've learned to create bridges between Python-based AI agents and ROS 2 robotic controllers. You've implemented:

1. Publisher-subscriber patterns for continuous control and monitoring
2. Service-based communication for discrete command and control
3. Action-based communication for complex, long-running behaviors
4. Behavior tree structures for organizing AI decision-making
5. Integrated applications combining multiple AI-robot communication patterns

These skills form the foundation for building sophisticated AI-driven robotic systems where digital intelligence is effectively connected to physical embodiment.

## Knowledge Check

1. When would you use topics vs services vs actions for AI-robot communication?
2. How do you handle timing and synchronization issues when bridging AI and ROS systems?
3. What are the advantages of using a behavior tree over a simple state machine?
4. How would you implement error recovery in an AI-robot bridge application?
5. What security considerations should be made when connecting AI systems to robots?

---

<nav>
  <div class="prev-next-pagination" layout="row" align="center" justify="between">
    <a href="/docs/modules/02-ros2-architecture-deep-dive"><- Previous: ROS 2 Architecture Deep Dive</a>
    <a href="/docs/modules/04-urdf-humanoid-robots">Next: URDF for Humanoid Robots -></a>
  </div>
</nav>