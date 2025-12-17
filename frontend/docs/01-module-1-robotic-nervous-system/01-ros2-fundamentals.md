

# 01-ROS 2 Fundamentals: Nodes, Topics, and Services

Welcome, aspiring roboticists! In this foundational chapter, we embark on our journey into the heart of modern robotics middleware: ROS 2 (Robot Operating System 2). ROS 2 is not merely an operating system; it's a flexible framework for writing robot software, providing a structured communication layer, tools, and libraries that simplify the complexities of robotics development. Its distributed nature and robust communication mechanisms make it ideal for building everything from simple mobile robots to intricate humanoid systems.

At its core, ROS 2 facilitates modularity, allowing different components of a robot system to operate independently while seamlessly exchanging information. This modularity is achieved through a set of fundamental concepts that we will explore in detail: Nodes, Topics, and Services. Understanding these concepts is paramount to designing, implementing, and debugging any ROS 2 application.

## 1. ROS 2 Nodes: The Brains of the Operation

In ROS 2, a **Node** is essentially an executable process that performs a specific computation. Think of it as a single-purpose program within your robotic ecosystem. For instance, you might have one node responsible for reading data from a LIDAR sensor, another for controlling motor movements, and yet another for performing high-level navigation planning.

The beauty of nodes lies in their independence. Each node can be written in different programming languages (Python, C++, etc.), run on separate machines, and even be developed by different teams. This distributed architecture enhances robustness, as the failure of one node does not necessarily bring down the entire system. Nodes communicate with each other using the ROS 2 communication mechanisms, forming a networked graph of processes.

Let's look at a simple Python example of a ROS 2 node. This node will simply print a message periodically.

```python
import rclpy
from rclpy.node import Node
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Node name is 'simple_publisher'
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('SimplePublisher node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive until Ctrl+C
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run this node, you would first need to set up a ROS 2 environment and then execute this Python script.

## 2. ROS 2 Topics: The Data Pipelines

While nodes are the active computational units, **Topics** are the channels through which data flows between them. Topics implement a publish-subscribe model, a highly efficient one-to-many asynchronous communication paradigm. A node can "publish" data to a topic, and any number of other nodes can "subscribe" to that same topic to receive the data.

Consider our LIDAR sensor example. The LIDAR node would publish scan data to a topic named `/laser_scan`. A navigation node, a mapping node, and a visualization node could all subscribe to `/laser_scan` to receive this data simultaneously, without the publisher needing to know about its subscribers.

Data published on a topic consists of ROS 2 messages, which are strongly typed data structures. ROS 2 provides a rich set of standard message types for common robotics data (e.g., `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist`). You can also define your own custom message types when needed.

Let's extend our previous Python example to show a subscriber node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 String message type

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter', # Topic name to subscribe to
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('SimpleSubscriber node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To observe this in action, you would run the `simple_publisher` node in one terminal and the `simple_subscriber` node in another. You'd see the subscriber receiving and printing messages from the publisher.

## 3. ROS 2 Services: The Request-Response Interaction

While topics are excellent for continuous, asynchronous data streams, sometimes you need a synchronous request-response interaction. For these scenarios, ROS 2 provides **Services**. Services are a remote procedure call (RPC) mechanism where a client node sends a request to a service server node, and the server processes the request and sends back a single response.

Think of a robot arm. You might have a service named `/move_arm` that takes a target joint configuration as a request and returns a boolean response indicating whether the movement was successful. The client waits for this response before proceeding.

Services are defined by a pair of message types: one for the request and one for the response. Like topics, ROS 2 provides standard service types, and you can define custom ones.

Here's a simplified Python example demonstrating a ROS 2 service:

**Service Server:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # A standard service type

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts service server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        response = add_two_ints_client.send_request(a, b)
        add_two_ints_client.get_logger().info(
            f'Result of add_two_ints: for {a} + {b} = {response.sum}')
    else:
        add_two_ints_client.get_logger().info('Usage: ros2 run <package_name> add_two_ints_client A B')
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run these, start the `add_two_ints_server` node, then in a separate terminal, execute the `add_two_ints_client` with two integer arguments. The client will send a request, and the server will return the sum.

## Conclusion

Nodes, Topics, and Services form the bedrock of ROS 2 communication. Nodes encapsulate specific functionalities, topics provide flexible data streams via a publish-subscribe model, and services enable synchronous request-response interactions. Mastering these concepts is crucial for building robust, modular, and scalable robotic applications. As you progress, you'll find these primitives invaluable for orchestrating the complex behaviors of autonomous systems.

In the next chapter, we'll delve into how Python agents can seamlessly integrate with ROS 2 controllers using the `rclpy` client library, bridging the gap between high-level AI and low-level robot actuation. Stay tuned!