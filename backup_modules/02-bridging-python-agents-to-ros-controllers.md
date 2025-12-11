---
title: "Bridging Python Agents to ROS Controllers using rclpy"
sidebar_label: "Python to ROS Bridge"
description: "Learn how to connect Python AI agents with ROS 2 controllers using the rclpy library"
---

# Bridging Python Agents to ROS Controllers using rclpy

## Overview

This module covers the essential techniques for connecting Python-based agents and algorithms to ROS 2 controllers. In humanoid robotics, this bridge is crucial for connecting high-level AI decision-making with low-level motor control and sensor processing. We'll explore how Python agents can communicate with ROS 2 nodes using the rclpy library, which is the native Python client library for ROS 2.

## Learning Objectives

By the end of this module, you will be able to:

1. Use rclpy to create ROS 2 nodes in Python
2. Implement publishers and subscribers for bidirectional communication
3. Create and use services for synchronous communication with robotic systems
4. Implement actions for long-running tasks with feedback
5. Bridge AI algorithms implemented in Python with ROS 2 controllers

## Prerequisites

Before starting this module, you should have:

- Completed Module 1: "The Robotic Nervous System (ROS 2)"
- Basic understanding of Python programming
- Familiarity with ROS 2 concepts (nodes, topics, services)
- Basic knowledge of robotic control concepts

Estimated duration: 15 hours

## Table of Contents

1. [Introduction to rclpy](#introduction-to-rclpy)
2. [Creating Python Nodes](#creating-python-nodes)
3. [Publishing Data to ROS 2](#publishing-data-to-ros-2)
4. [Subscribing to ROS 2 Topics](#subscribing-to-ros-2-topics)
5. [Using Services with Python](#using-services-with-python)
6. [Using Actions with Python](#using-actions-with-python)
7. [Advanced: Connecting AI Algorithms to ROS Controllers](#advanced-connecting-ai-algorithms-to-ros-controllers)
8. [Hands-on Exercises](#hands-on-exercises)
9. [Summary and Next Steps](#summary-and-next-steps)

---

## Introduction to rclpy

The Robot Client Library for Python (rclpy) is the native Python library for ROS 2. It allows Python programs to connect to a ROS graph as nodes, enabling communication with other ROS nodes through topics, services, and actions.

### Installing rclpy

rclpy comes as part of the standard ROS 2 installation. To verify it's available in your Python environment:

```python
import rclpy
print(rclpy.__version__)
```

### Basic Node Structure

Every ROS 2 node using rclpy follows a similar structure:

```python
import rclpy
from rclpy.node import Node

class MyPythonNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Python Nodes

When bridging Python agents to ROS controllers, your Python code becomes a node in the ROS graph. This enables direct communication with other components in your humanoid robot system.

### Node Best Practices for AI Integration

When developing Python agents that connect to ROS controllers, consider these best practices:

1. **Clear Purpose**: Each node should have a single, well-defined purpose
2. **Proper Logging**: Log important events for debugging
3. **Graceful Shutdown**: Handle interrupts properly to clean up resources
4. **Parameter Handling**: Use ROS parameters for configuration

Example of a well-structured Python agent node:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PythonAgentBridge(Node):
    """
    An example node that bridges Python-based AI agents
    with ROS 2 controllers in a humanoid robot system.
    """
    
    def __init__(self):
        super().__init__('python_agent_bridge')
        
        # Declare parameters for configuration
        self.declare_parameter('robot_speed', 0.5)
        self.declare_parameter('safety_distance', 0.7)
        self.declare_parameter('control_frequency', 10)  # Hz
        
        # Get parameter values
        self.robot_speed = self.get_parameter('robot_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        control_freq = self.get_parameter('control_frequency').value
        
        # Create publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.sensor_callback,
            10
        )
        
        # Create a timer for periodic AI processing
        timer_period = 1.0 / control_freq  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.latest_scan = None
        self.get_logger().info(
            f'Python Agent Bridge initialized with speed={self.robot_speed}, '
            f'safety_distance={self.safety_distance}, frequency={control_freq}Hz'
        )

    def sensor_callback(self, msg):
        """
        Process incoming sensor data
        """
        self.latest_scan = msg
        self.get_logger().debug(f'Received sensor scan with {len(msg.ranges)} readings')

    def control_loop(self):
        """
        Main control loop where AI decisions are made and sent to ROS controllers
        """
        if self.latest_scan is None:
            self.get_logger().debug('Waiting for sensor data...')
            return
        
        # Implement AI logic here
        velocity_command = self.make_decision(self.latest_scan)
        
        # Publish the command to ROS controllers
        self.velocity_publisher.publish(velocity_command)

    def make_decision(self, sensor_data):
        """
        The AI decision-making logic that converts sensor data
        to velocity commands for the robot
        """
        # Simplified AI logic: avoid obstacles
        min_distance = min([r for r in sensor_data.ranges if r > 0 and r < float('inf')], default=float('inf'))
        
        cmd_vel = Twist()
        if min_distance > self.safety_distance:
            # Safe to move forward
            cmd_vel.linear.x = self.robot_speed
            cmd_vel.angular.z = 0.0
        else:
            # Turn to avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
        
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    bridge_node = PythonAgentBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Shutting down Python Agent Bridge')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishing Data to ROS 2

Publishers send messages to topics in the ROS graph. This is how your Python agent sends commands or data to other ROS components.

### Publisher Best Practices

1. **Appropriate QoS Settings**: Use appropriate Quality of Service settings based on your use case
2. **Consistent Message Types**: Always publish the correct message type for the topic
3. **Error Handling**: Handle scenarios where publishing might fail

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PublisherAgent(Node):
    def __init__(self):
        super().__init__('publisher_agent')
        
        # Create QoS profile for reliable command publishing
        cmd_qoss_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, '/robot/cmd_vel', cmd_qos_profile)
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, '/agent/status', 10)
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_commands)  # 10 Hz
        
        self.command_counter = 0

    def publish_commands(self):
        """
        Publish robot commands based on AI decisions
        """
        # Create velocity command based on AI decision
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_vel.angular.z = 0.0  # No turning
        
        # Publish the command
        self.cmd_publisher.publish(cmd_vel)
        
        # Also publish status update
        status_msg = String()
        status_msg.data = f'Published command #{self.command_counter}'
        self.status_publisher.publish(status_msg)
        
        self.command_counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher_agent = PublisherAgent()
    
    try:
        rclpy.spin(publisher_agent)
    except KeyboardInterrupt:
        publisher_agent.get_logger().info('Shutting down publisher agent')
    finally:
        publisher_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscribing to ROS 2 Topics

Subscribers receive messages from topics in the ROS graph. This is how your Python agent gets data from sensors and other ROS components.

### Subscriber Best Practices

1. **Appropriate Callbacks**: Process messages quickly in callbacks
2. **Buffer Management**: Use appropriate queue sizes
3. **Message Filtering**: If needed, filter messages based on timestamps
4. **Threading Considerations**: Be aware of threading implications

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

class SubscriberAgent(Node):
    def __init__(self):
        super().__init__('subscriber_agent')
        
        # Initialize CvBridge for image processing
        self.cv_bridge = CvBridge()
        
        # Subscribe to various sensor topics
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            20  # Larger queue for sensor data
        )
        
        # Store latest sensor readings
        self.latest_odom = None
        self.latest_scan = None
        self.scan_count = 0

    def odom_callback(self, msg):
        """
        Process odometry data
        """
        self.latest_odom = msg
        pos = msg.pose.pose.position
        self.get_logger().debug(
            f'Odom received: position=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
        )

    def scan_callback(self, msg):
        """
        Process laser scan data for obstacle detection
        """
        self.latest_scan = msg
        min_dist = min([r for r in msg.ranges if r > 0 and r < float('inf')], default=float('inf'))
        
        self.get_logger().debug(f'Scan #{self.scan_count}: min distance={min_dist:.2f}m')
        self.scan_count += 1

def main(args=None):
    rclpy.init(args=args)
    subscriber_agent = SubscriberAgent()
    
    try:
        rclpy.spin(subscriber_agent)
    except KeyboardInterrupt:
        subscriber_agent.get_logger().info('Shutting down subscriber agent')
    finally:
        subscriber_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using Services with Python

Services provide synchronous request-response communication. This is useful when your Python AI agent needs to query information from or command other nodes in a synchronous way.

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import time

class ServiceClientAgent(Node):
    def __init__(self):
        super().__init__('service_client_agent')
        
        # Create a client for the add_two_ints service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = AddTwoInts.Request()
        
        # Timer to periodically call the service
        self.timer = self.create_timer(2.0, self.make_service_request)
        self.request_counter = 0

    def make_service_request(self):
        """
        Make a request to the service
        """
        self.request.a = self.request_counter
        self.request.b = self.request_counter + 1
        
        # Call the service asynchronously
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.service_response_callback)
        
        self.request_counter += 1

    def service_response_callback(self, future):
        """
        Process the response from the service
        """
        try:
            response = future.result()
            self.get_logger().info(
                f'Received service response: {self.request.a} + {self.request.b} = {response.sum}'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    service_client_agent = ServiceClientAgent()
    
    try:
        rclpy.spin(service_client_agent)
    except KeyboardInterrupt:
        service_client_agent.get_logger().info('Shutting down service client agent')
    finally:
        service_client_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String
import time

class ServiceServerAgent(Node):
    def __init__(self):
        super().__init__('service_server_agent')
        
        # Create a service
        self.srv = self.create_service(AddTwoInts, 'ai_decision_service', self.decision_callback)
        
        # Also create a publisher for logging decisions
        self.log_publisher = self.create_publisher(String, '/ai_decisions_log', 10)
        
        self.decision_counter = 0
        self.get_logger().info('AI Decision Service started')

    def decision_callback(self, request, response):
        """
        Process service request and make AI decision
        """
        # Simulate AI decision process
        decision_value = self.simulate_ai_decision(request.a, request.b)
        response.sum = decision_value
        
        # Log the decision
        log_msg = String()
        log_msg.data = f'Decision #{self.decision_counter}: input=({request.a}, {request.b}), decision={decision_value}'
        self.log_publisher.publish(log_msg)
        
        self.decision_counter += 1
        self.get_logger().info(f'Made decision: {response.sum}')
        
        return response

    def simulate_ai_decision(self, input_a, input_b):
        """
        Simulated AI decision-making logic
        """
        # In a real implementation, this would run actual AI algorithms
        time.sleep(0.1)  # Simulate processing time
        return input_a + input_b + 10  # Add 10 as our "AI" contribution

def main(args=None):
    rclpy.init(args=args)
    server_agent = ServiceServerAgent()
    
    try:
        rclpy.spin(server_agent)
    except KeyboardInterrupt:
        server_agent.get_logger().info('Shutting down service server agent')
    finally:
        server_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using Actions with Python

Actions are ideal for long-running tasks that require feedback and can be cancelled. In humanoid robotics, this includes navigation, manipulation, and other goal-oriented behaviors.

### Action Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClientAgent(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client_agent')
        
        # Create an action client
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci_sequence')
        
        # Call action after startup
        self.timer = self.create_timer(3.0, self.send_goal)
        self.goal_sent = False

    def send_goal(self):
        """
        Send a goal to the action server
        """
        if self.goal_sent:
            return  # Don't send another goal if one is already active
        
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Calculate 10 numbers in the sequence
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        """
        Handle the response from the action server when the goal is accepted/rejected
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_sent = False
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server during execution
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        """
        Handle the final result from the action server
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        
        self.goal_sent = False  # Allow sending another goal

def main(args=None):
    rclpy.init(args=args)
    action_client_agent = FibonacciActionClientAgent()
    
    try:
        rclpy.spin(action_client_agent)
    except KeyboardInterrupt:
        action_client_agent.get_logger().info('Shutting down action client agent')
    finally:
        action_client_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced: Connecting AI Algorithms to ROS Controllers

For humanoid robotics, we often need to connect sophisticated AI algorithms to ROS controllers. Here's an example of how to connect a reinforcement learning agent with ROS 2:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class RLBridgeAgent(Node):
    """
    Example of how to connect a reinforcement learning algorithm to ROS 2 controllers
    """
    
    def __init__(self):
        super().__init__('rl_bridge_agent')
        
        # Publishers and subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reward_publisher = self.create_publisher(Float32, '/current_reward', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.rl_control_loop)  # 10 Hz
        
        # RL Agent state (simplified for example)
        self.latest_scan = None
        self.is_training = True
        self.epsilon = 0.1  # Exploration parameter
        self.q_table = {}  # Simplified Q-table for this example
        self.current_state = None
        self.previous_state = None
        self.previous_action = None

    def scan_callback(self, msg):
        """
        Process laser scan data and discretize into states for RL algorithm
        """
        self.latest_scan = msg
        
        # Discretize laser scan into a state representation
        discretized_state = self.discretize_scan(msg)
        self.current_state = discretized_state

    def discretize_scan(self, scan_msg):
        """
        Convert continuous laser scan to discrete state representation
        This is a simplified example - real discretization would be more sophisticated
        """
        # Group laser readings into sectors
        num_sectors = 8
        sector_size = len(scan_msg.ranges) // num_sectors
        
        sectors = []
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = min((i + 1) * sector_size, len(scan_msg.ranges))
            sector_avg = sum(scan_msg.ranges[start_idx:end_idx]) / (end_idx - start_idx)
            sectors.append(sector_avg)
        
        # Discretize each sector into "clear" (1), "near" (0.5), or "near" (0)
        discretized = []
        for dist in sectors:
            if dist > 1.0:  # More than 1m is clear
                discretized.append(1)
            elif dist > 0.3:  # Between 0.3m and 1m is medium
                discretized.append(0.5)
            else:  # Less than 0.3m is obstacle
                discretized.append(0)
        
        # Convert to tuple as dict key (needs to be hashable)
        return tuple(discretized)

    def rl_control_loop(self):
        """
        Main control loop for RL agent
        """
        if self.latest_scan is None:
            return  # Wait for sensor data
        
        # If this is the first iteration, initialize state
        if self.current_state is None:
            return
        
        # Get action from RL algorithm
        action = self.get_action(self.current_state)
        
        # Convert action to Twist message for ROS
        cmd_vel = self.convert_action_to_velocity(action)
        
        # Publish command
        self.velocity_publisher.publish(cmd_vel)
        
        # If not the first iteration, update Q-table
        if self.previous_state is not None and self.previous_action is not None:
            reward = self.calculate_reward(self.current_state)
            self.update_rl_algorithm(self.previous_state, self.previous_action, reward, self.current_state)
            
            # Publish reward for monitoring
            reward_msg = Float32()
            reward_msg.data = reward
            self.reward_publisher.publish(reward_msg)
        
        # Store this iteration's data for next iteration
        self.previous_state = self.current_state
        self.previous_action = action

    def get_action(self, state):
        """
        Get action from RL algorithm based on current state
        This is a simplified example using epsilon-greedy
        """
        if np.random.rand() < self.epsilon:
            # Random action (exploration)
            return np.random.choice(3)  # 0: left, 1: forward, 2: right
        else:
            # Greedy action based on Q-table (exploitation)
            if state not in self.q_table:
                # Initialize Q-values if state not seen before
                self.q_table[state] = [0.0, 0.0, 0.0]
            
            # Return action with highest Q-value
            return np.argmax(self.q_table[state])

    def convert_action_to_velocity(self, action):
        """
        Convert discrete action to Twist velocity command
        """
        cmd_vel = Twist()
        
        if action == 0:  # Turn left
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.3
        elif action == 1:  # Go forward
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
        elif action == 2:  # Turn right
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = -0.3
        
        return cmd_vel

    def calculate_reward(self, state):
        """
        Calculate reward based on current state
        """
        # Reward based on distance from obstacles
        min_distance = min(state) if state else 0
        
        if min_distance == 0:  # Collision or near collision
            return -100.0
        elif min_distance < 0.5:  # Too close to obstacles
            return -1.0
        elif min_distance > 1.0:  # Good clear path ahead
            return 1.0
        else:  # Medium distance (acceptable)
            return 0.1

    def update_rl_algorithm(self, prev_state, action, reward, curr_state):
        """
        Update the RL algorithm based on experience
        """
        learning_rate = 0.1
        discount_factor = 0.9
        
        # Initialize state entries if needed
        if prev_state not in self.q_table:
            self.q_table[prev_state] = [0.0, 0.0, 0.0]
        if curr_state not in self.q_table:
            self.q_table[curr_state] = [0.0, 0.0, 0.0]
        
        # Update Q-value for the previous state-action pair
        old_value = self.q_table[prev_state][action]
        future_rewards = discount_factor * max(self.q_table[curr_state])
        
        self.q_table[prev_state][action] = old_value + learning_rate * (reward + future_rewards - old_value)

def main(args=None):
    rclpy.init(args=args)
    rl_agent = RLBridgeAgent()
    
    try:
        rclpy.spin(rl_agent)
    except KeyboardInterrupt:
        rl_agent.get_logger().info('Shutting down RL bridge agent')
    finally:
        rl_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 1: Simple Publisher-Subscriber Bridge

Create a Python agent that acts as a bridge between two topics. It should subscribe to a topic that publishes sensor data, run a simple AI decision-making algorithm (like detecting if the sensor value is within a safe range), and publish velocity commands accordingly.

### Exercise 2: Service-Based AI Planner

Create a Python node that acts as a service server. Clients can send position requests, and your AI algorithm should determine a safe path to that position. Implement a simple path planning algorithm that respects obstacles published in a map topic.

### Exercise 3: Action-Based Manipulation Controller

Create an action server that accepts manipulation goals (like "pick up object at position x,y,z") and uses an AI algorithm to determine the sequence of joint angles needed to accomplish the task.

## Summary and Next Steps

In this module, we explored how Python agents can be connected to ROS 2 controllers using rclpy. We covered all the major communication paradigms:

- **Publishers/Subscribers**: For streaming sensor data and commands
- **Services**: For synchronous request-response communication
- **Actions**: For goal-oriented tasks with feedback

These communication patterns are fundamental to connecting AI algorithms to robotic systems, and they form the backbone of humanoid robotics applications where high-level decision making needs to seamlessly interact with low-level control.

The concepts in this module will be essential as we move forward to implement more sophisticated AI-integrated systems in later modules, including perception systems, path planning, and voice command processing.

## Key Takeaways

- Python agents can effectively bridge the gap between AI algorithms and ROS controllers
- Each communication pattern (topics, services, actions) has its appropriate use case
- When implementing AI in robotics, consider the timing requirements and communication patterns
- Proper error handling and graceful degradation are crucial in robotic systems
- The QoS settings significantly impact system reliability for different use cases

## Self-Assessment Questions

1. When would you use a service instead of a topic for AI-robot communication?
2. How do actions handle the challenge of long-running operations differently than services?
3. What considerations should be made when bridging an AI algorithm to ROS controllers?
4. How does the QoS profile affect the reliability of communication in a robotic system?

---

<nav>
  <div class="prev-next-pagination" layout="row" align="center" justify="between">
    <a href="/docs/modules/01-the-robotic-nervous-system-ros2"><- Previous: The Robotic Nervous System (ROS 2)</a>
    <a href="/docs/modules/03-urdf-for-humanoid-robots">Next: URDF for Humanoid Robots -></a>
  </div>
</nav>