#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Node
Subscribes to /robot/sensor_data and processes readings

Usage:
    ros2 run <package_name> simple_subscriber.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SimpleSensorSubscriber(Node):
    """Processes incoming sensor data and logs statistics"""

    def __init__(self):
        super().__init__('simple_sensor_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            Float32,
            '/robot/sensor_data',
            self.sensor_callback,
            10  # QoS history depth
        )

        # Statistics tracking
        self.reading_count = 0
        self.sum_readings = 0.0
        self.min_reading = float('inf')
        self.max_reading = float('-inf')

        self.get_logger().info('Sensor subscriber node started')

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        reading = msg.data

        # Update statistics
        self.reading_count += 1
        self.sum_readings += reading
        self.min_reading = min(self.min_reading, reading)
        self.max_reading = max(self.max_reading, reading)

        avg = self.sum_readings / self.reading_count

        self.get_logger().info(
            f'Received: {reading:.2f} | '
            f'Avg: {avg:.2f} | '
            f'Min: {self.min_reading:.2f} | '
            f'Max: {self.max_reading:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
