#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Node
Publishes sensor data to /robot/sensor_data topic

Usage:
    ros2 run <package_name> simple_publisher.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SimpleSensorPublisher(Node):
    """Publishes simulated sensor readings at 10 Hz"""

    def __init__(self):
        super().__init__('simple_sensor_publisher')

        # Create publisher for sensor data
        self.publisher_ = self.create_publisher(
            Float32,
            '/robot/sensor_data',
            10  # QoS history depth
        )

        # Create timer for periodic publishing (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Sensor publisher node started')

    def timer_callback(self):
        """Called every 100ms to publish sensor data"""
        msg = Float32()
        # Simulate sensor reading between 20.0 and 25.0
        msg.data = 20.0 + random.random() * 5.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
