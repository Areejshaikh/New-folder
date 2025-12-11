"""
Basic ROS 2 Examples for the PAHR Book Curriculum

This module implements basic ROS 2 examples following the quickstart guide.
These examples demonstrate fundamental ROS 2 concepts for humanoid robotics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class HelloWorldPublisher(Node):
    """
    Basic ROS 2 publisher example - Hello World
    """
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher = self.create_publisher(String, 'hello_world', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class HelloWorldSubscriber(Node):
    """
    Basic ROS 2 subscriber example - Hello World
    """
    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_world',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


class TurtleBotController(Node):
    """
    Example of a simple robot controller that moves in a square pattern
    """
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.move_forward_time = 4  # seconds to move forward
        self.turn_time = 6  # seconds to turn (90-degree turn)
        self.state = 'forward'  # 'forward' or 'turn'
        self.state_start_time = self.get_clock().now()
        self.move_counter = 0  # Count movements (0-3 for square)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

    def control_loop(self):
        msg = Twist()
        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9  # Convert to seconds

        if self.state == 'forward':
            msg.linear.x = 0.2  # Move forward at 0.2 m/s
            msg.angular.z = 0.0

            if elapsed >= self.move_forward_time:
                # Switch to turn state
                self.state = 'turn'
                self.state_start_time = current_time
        else:  # self.state == 'turn'
            msg.linear.x = 0.0
            msg.angular.z = 0.26  # Turn at 0.26 rad/s (approx 90 degrees in 6 seconds)

            if elapsed >= self.turn_time:
                # Switch back to forward state
                self.state = 'forward'
                self.state_start_time = current_time
                self.move_counter += 1

                if self.move_counter >= 4:  # Complete one square
                    self.move_counter = 0  # Reset for next square

        self.cmd_vel_publisher.publish(msg)


class LaserScanSubscriber(Node):
    """
    Example of subscribing to laser scan data for obstacle detection
    """
    def __init__(self):
        super().__init__('laser_scan_subscriber')

        # Subscribe to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create publisher to share obstacle information
        self.obstacle_publisher = self.create_publisher(String, '/obstacle_alert', 10)

    def scan_callback(self, msg):
        # Find minimum range (closest obstacle)
        if len(msg.ranges) > 0:
            # Filter out invalid ranges (inf or nan)
            valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r))]

            if valid_ranges:
                min_range = min(valid_ranges)

                if min_range < 0.5:  # Less than half a meter
                    self.get_logger().warn(f'OBSTACLE DETECTED: {min_range:.2f} meters away')

                    # Publish alert
                    alert_msg = String()
                    alert_msg.data = f'Obstacle detected at {min_range:.2f} meters'
                    self.obstacle_publisher.publish(alert_msg)
                else:
                    self.get_logger().info(f'Clear path detected: {min_range:.2f} meters')
            else:
                self.get_logger().info('No valid ranges available')


class SimpleServiceServer(Node):
    """
    Example of a simple service server
    """
    def __init__(self):
        super().__init__('simple_service_server')

        from example_interfaces.srv import AddTwoInts
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response


class SimpleServiceClient(Node):
    """
    Example of a simple service client
    """
    def __init__(self):
        super().__init__('simple_service_client')

        from example_interfaces.srv import AddTwoInts
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

        # Set up a timer to call the service periodically
        self.timer = self.create_timer(3.0, self.send_request)

    def send_request(self):
        self.req.a = 42
        self.req.b = 24
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Result of add_two_ints: {response.sum}')


def run_hello_world_demo():
    """
    Run the hello world publisher/subscriber demo
    """
    rclpy.init()

    publisher = HelloWorldPublisher()
    subscriber = HelloWorldSubscriber()

    # Use a MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    try:
        print("Running Hello World Publisher/Subscriber Demo...")
        executor.spin()
    except KeyboardInterrupt:
        print("Demo stopped by user")
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()


def run_robot_controller_demo():
    """
    Run the robot controller demo
    """
    rclpy.init()

    controller = TurtleBotController()

    try:
        print("Running TurtleBot Controller Demo...")
        print("Robot will move in a square pattern")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Demo stopped by user")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


def run_laser_scan_demo():
    """
    Run the laser scan subscriber demo
    """
    rclpy.init()

    scanner = LaserScanSubscriber()

    try:
        print("Running Laser Scan Subscriber Demo...")
        print("Detecting obstacles from /scan topic...")
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        print("Demo stopped by user")
    finally:
        scanner.destroy_node()
        rclpy.shutdown()


def run_service_demo():
    """
    Run the service server/client demo
    """
    rclpy.init()

    server = SimpleServiceServer()
    client = SimpleServiceClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    try:
        print("Running Service Server/Client Demo...")
        executor.spin()
    except KeyboardInterrupt:
        print("Demo stopped by user")
    finally:
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()


def main():
    """
    Main function to demonstrate basic ROS 2 examples
    """
    print("PAHR Book - Basic ROS 2 Examples")
    print("=================================")

    print("\nThis module includes the following examples:")
    print("1. Hello World Publisher/Subscriber")
    print("2. Robot Controller (Square Movement)")
    print("3. Laser Scan Subscriber (Obstacle Detection)")
    print("4. Service Server/Client")

    print("\nTo run an example, uncomment the appropriate section below:")

    # Uncomment to run specific demos:
    # run_hello_world_demo()
    # run_robot_controller_demo()
    # run_laser_scan_demo()
    # run_service_demo()

    print("\nAlternatively, here's a summary of the code structure:")

    example_code = '''
# Hello World Publisher
class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher = self.create_publisher(String, 'hello_world', 10)
        self.timer = self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# Hello World Subscriber
class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_world',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
'''
    print(example_code)


if __name__ == '__main__':
    main()