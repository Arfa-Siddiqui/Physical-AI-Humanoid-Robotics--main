#!/usr/bin/env python3
"""
Simple ROS 2 Service Server
Provides a service to compute robot kinematics

Usage:
    ros2 run <package_name> simple_service_server.py

Call service with:
    ros2 service call /compute_distance std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import math


class KinematicsServiceServer(Node):
    """Service that computes distance traveled by robot"""

    def __init__(self):
        super().__init__('kinematics_service')

        # Robot position
        self.x = 0.0
        self.y = 0.0
        self.total_distance = 0.0

        # Create service
        self.srv = self.create_service(
            Trigger,
            '/compute_distance',
            self.compute_distance_callback
        )

        # Simulate robot movement
        self.timer = self.create_timer(1.0, self.update_position)

        self.get_logger().info('Kinematics service ready')

    def update_position(self):
        """Simulate robot moving in a circle"""
        import random
        old_x, old_y = self.x, self.y

        # Update position (simple random walk)
        self.x += random.uniform(-0.5, 0.5)
        self.y += random.uniform(-0.5, 0.5)

        # Calculate distance traveled
        dx = self.x - old_x
        dy = self.y - old_y
        distance = math.sqrt(dx**2 + dy**2)
        self.total_distance += distance

    def compute_distance_callback(self, request, response):
        """Service callback - return current distance traveled"""
        response.success = True
        response.message = (
            f'Robot position: ({self.x:.2f}, {self.y:.2f}), '
            f'Total distance: {self.total_distance:.2f}m'
        )

        self.get_logger().info(f'Service called: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    server = KinematicsServiceServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
