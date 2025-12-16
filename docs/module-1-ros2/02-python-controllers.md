---
sidebar_position: 2
---

# Building Python Controllers with rclpy

Python is the lingua franca of AI and robotics research, making `rclpy` (ROS Client Library for Python) the go-to choice for rapid prototyping and integrating machine learning models. This chapter teaches you to write ROS 2 nodes in Python, from simple publishers to complex control loops.

## Setting Up a ROS 2 Python Package

First, create a workspace and package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_controller
```

This generates:

```
my_robot_controller/
├── my_robot_controller/
│   └── __init__.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource/
```

**Key Files:**
- `package.xml`: Package metadata and dependencies
- `setup.py`: Python package configuration and entry points
- `my_robot_controller/`: Source code directory

## Minimal Publisher: Hello World

Create `my_robot_controller/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Breakdown:**
- **`super().__init__('talker')`**: Initialize node with name "talker"
- **`create_publisher(String, 'chatter', 10)`**: Publish String messages on topic "chatter" with queue size 10
- **`create_timer(1.0, callback)`**: Call `timer_callback` every 1.0 seconds
- **`rclpy.spin(node)`**: Keep node alive, processing callbacks

**Register Entry Point** in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_controller.talker:main',
    ],
},
```

**Build and Run:**

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 run my_robot_controller talker
```

## Subscriber: Receiving Messages

Create `my_robot_controller/listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run Listener:**

```bash
ros2 run my_robot_controller listener
```

With talker running in another terminal, you'll see:

```
[INFO] [listener]: I heard: "Hello World: 5"
```

## Velocity Publisher: Controlling a Robot

Humanoid robots move via velocity commands (`geometry_msgs/Twist`). Create `my_robot_controller/velocity_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5   # Move forward at 0.5 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**: This publishes to `/cmd_vel` at 10 Hz, commanding the robot to move forward. Gazebo or a real robot subscribes to this topic to execute motion.

## Parameters: Configurable Nodes

Make velocity configurable via parameters:

```python
class ConfigurableVelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Declare parameters with defaults
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.0)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_speed').value
        msg.angular.z = self.get_parameter('angular_speed').value
        self.publisher_.publish(msg)
```

**Set Parameter at Launch:**

```bash
ros2 run my_robot_controller velocity_controller --ros-args -p linear_speed:=1.0
```

## Service Client: Requesting Computation

Create a service client to call `/add_two_ints`:

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    future = node.send_request(10, 20)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(f'Result: {response.sum}')
    node.destroy_node()
    rclpy.shutdown()
```

## Action Client: Goal-Based Tasks

Actions enable feedback and cancellation. Example: navigating to a pose.

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
```

## Logging Best Practices

ROS 2 provides severity levels:

```python
self.get_logger().debug('Detailed diagnostic info')
self.get_logger().info('Normal operation message')
self.get_logger().warn('Unexpected but handled situation')
self.get_logger().error('Error occurred, but node continues')
self.get_logger().fatal('Critical failure, node should stop')
```

**View Logs:**

```bash
ros2 run my_robot_controller talker --ros-args --log-level DEBUG
```

## Executors: Managing Multiple Nodes

Run multiple nodes in one process:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    talker = TalkerNode()
    listener = ListenerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(talker)
    executor.add_node(listener)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        talker.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
```

## Debugging Tips

### Inspect Topics

```bash
# List all topics
ros2 topic list

# Show topic data type
ros2 topic info /chatter

# Print messages in real-time
ros2 topic echo /chatter
```

### Measure Frequency

```bash
# Check publish rate
ros2 topic hz /cmd_vel
```

### Visualize Data

```bash
# Plot numeric data
rqt_plot /joint_states/position[0]
```

## Practical Example: Teleoperation

Combine keyboard input with velocity publishing:

```python
import sys
import tty
import termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("Use WASD to move. Q to quit.")
        while True:
            key = self.get_key()
            msg = Twist()

            if key == 'w':
                msg.linear.x = 0.5
            elif key == 's':
                msg.linear.x = -0.5
            elif key == 'a':
                msg.angular.z = 0.5
            elif key == 'd':
                msg.angular.z = -0.5
            elif key == 'q':
                break

            self.publisher_.publish(msg)
```

This enables manual control of a simulated or real robot.

## Conclusion

You've learned to build ROS 2 nodes in Python: publishers, subscribers, services, actions, and parameters. These patterns form the building blocks of robotic applications. In the next chapter, we'll model a humanoid robot using URDF, enabling simulation and visualization.

## References

- Open Robotics. (2024). "rclpy API Documentation." Retrieved from https://docs.ros.org/en/humble/p/rclpy/
- Koubaa, A. (Ed.). (2020). *Robot Operating System (ROS): The Complete Reference (Volume 5)*. Springer.
