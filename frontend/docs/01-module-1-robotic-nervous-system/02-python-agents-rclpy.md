

# 02-Bridging Python Agents to ROS Controllers using rclpy

Welcome back, future robot architects! In the previous chapter, we laid the groundwork by understanding the core communication primitives of ROS 2: Nodes, Topics, and Services. Now, we'll elevate our discussion to bridge the gap between high-level AI algorithms, often implemented in Python, and the low-level robotic control systems that typically reside within the ROS 2 framework. This integration is crucial for giving our intelligent agents the ability to perceive their environment, make decisions, and exert physical control over a robot.

The primary tool for this integration in Python is `rclpy`, the Python client library for ROS 2. `rclpy` provides a Pythonic interface to all the core ROS 2 functionalities, allowing us to create nodes, publish to topics, subscribe to topics, and implement services using Python's familiar syntax and extensive ecosystem of libraries for AI, machine learning, and data processing.

## 1. The Role of Python Agents in Robotics

Python has become the lingua franca for AI and machine learning development due to its rich libraries (TensorFlow, PyTorch, scikit-learn), ease of use, and rapid prototyping capabilities. In robotics, Python agents are often responsible for:

*   **Perception:** Processing sensor data (e.g., computer vision with OpenCV, natural language processing for human-robot interaction).
*   **Decision-Making:** Implementing high-level behaviors, path planning, reinforcement learning algorithms, and cognitive architectures.
*   **Task Planning:** Decomposing complex goals into a sequence of executable robot actions.
*   **User Interface:** Providing a scripting interface or visualization tools for human operators.

However, these Python agents typically don't directly manipulate robot hardware. Instead, they send commands to and receive feedback from ROS 2 controllers, which are often implemented in C++ for performance-critical, real-time operations. `rclpy` acts as the vital communication bridge.

## 2. Setting Up an rclpy Environment

Before diving into code, ensure your ROS 2 environment is sourced. For Python development, you'll typically be working within a ROS 2 workspace.

**Creating a Python Package:**

```bash
# Navigate to your ROS 2 workspace src directory
cd ~/ros2_ws/src

# Create a new Python package (e.g., my_python_agent)
ros2 pkg create --build-type ament_python my_python_agent

# Go into the package directory
cd my_python_agent
```

This command sets up the basic structure for your Python ROS 2 package, including a `setup.py` and `package.xml` for package management and dependencies.

## 3. Building an rclpy Publisher: Sending Commands

Let's imagine our Python agent needs to send velocity commands to a robot's base controller. This is a classic publish-subscribe scenario where the Python agent acts as a publisher to a `/cmd_vel` topic, and a low-level ROS 2 controller subscribes to it to execute the movement.

We'll use the `geometry_msgs/msg/Twist` message type, which is standard for conveying linear and angular velocities.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_speed = 0.2  # meters/second
        self.angular_speed = 0.5 # radians/second
        self.get_logger().info('VelocityCommander node started.')

    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = self.angular_speed
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Publishing linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_commander = VelocityCommander()
    rclpy.spin(velocity_commander)
    velocity_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To make this executable, save it as `velocity_commander.py` inside your `my_python_agent/my_python_agent/` directory (created by `ros2 pkg create`). Then, add an entry point in `setup.py`:

```python
# setup.py (excerpt)

entry_points={
    'console_scripts': [
        'velocity_commander = my_python_agent.velocity_commander:main',
    ],
},
```
After building your workspace (`colcon build`) and sourcing it, you can run `ros2 run my_python_agent velocity_commander`.

## 4. Building an rclpy Subscriber: Receiving Feedback

Conversely, a Python agent often needs to receive feedback from the robot, such as current odometry (position and orientation) or joint states. This is achieved by creating an `rclpy` subscriber.

Let's create a subscriber that listens to the `/odom` topic, which typically publishes `nav_msgs/msg/Odometry` messages.

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryListener(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('OdometryListener node started.')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f'Received Odometry: Position(x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}) '
            f'Orientation(x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    odometry_listener = OdometryListener()
    rclpy.spin(odometry_listener)
    odometry_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Save this as `odometry_listener.py` in the same Python package and add its entry point to `setup.py`. After rebuilding and sourcing, you can run `ros2 run my_python_agent odometry_listener` to see it in action, assuming there's another node publishing `Odometry` messages.

## 5. Services and Actions with rclpy

Beyond topics, `rclpy` also fully supports ROS 2 Services for request-response communication and Actions for long-running, pre-emptable tasks with periodic feedback. The patterns for creating clients and servers for services and actions in `rclpy` mirror the topic examples, utilizing `create_client`, `create_service`, `create_action_client`, and `create_action_server` methods respectively.

## Conclusion

`rclpy` is your essential toolkit for integrating Python-based AI and control logic with the ROS 2 ecosystem. By mastering the creation of `rclpy` nodes, publishers, and subscribers, you gain the power to develop sophisticated robotic behaviors where intelligent agents seamlessly interact with the robot's hardware and software components. This bridge enables your robots to become truly autonomous and intelligent.

Next up, we'll shift our focus to the physical description of robots. We'll explore URDF, the Unified Robot Description Format, and understand how it's used to accurately model the kinematic and dynamic properties of humanoid robots.