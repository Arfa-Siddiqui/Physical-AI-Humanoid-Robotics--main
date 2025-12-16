#!/usr/bin/env python3
"""
Humanoid Joint Controller Node
Controls humanoid robot joint positions via velocity commands

Usage:
    ros2 run <package_name> joint_controller.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math


class HumanoidJointController(Node):
    """
    Simple PD controller for humanoid robot joints
    Subscribes to desired joint positions and publishes velocity commands
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Joint names for humanoid robot
        self.joint_names = [
            'left_shoulder_pitch',
            'left_shoulder_roll',
            'left_elbow',
            'right_shoulder_pitch',
            'right_shoulder_roll',
            'right_elbow',
            'left_hip_pitch',
            'left_hip_roll',
            'left_knee',
            'right_hip_pitch',
            'right_hip_roll',
            'right_knee'
        ]

        # Current joint states
        self.current_positions = [0.0] * len(self.joint_names)
        self.current_velocities = [0.0] * len(self.joint_names)

        # Desired joint positions
        self.desired_positions = [0.0] * len(self.joint_names)

        # PD controller gains
        self.kp = 10.0  # Proportional gain
        self.kd = 2.0   # Derivative gain

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.desired_pos_sub = self.create_subscription(
            Float64MultiArray,
            '/desired_joint_positions',
            self.desired_position_callback,
            10
        )

        # Publisher for velocity commands
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_commands',
            10
        )

        # Control loop timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Joint controller started')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_positions[idx] = msg.position[i]
                if len(msg.velocity) > i:
                    self.current_velocities[idx] = msg.velocity[i]

    def desired_position_callback(self, msg):
        """Update desired joint positions"""
        if len(msg.data) == len(self.joint_names):
            self.desired_positions = list(msg.data)
            self.get_logger().info('Updated desired positions')

    def control_loop(self):
        """PD control loop - compute and publish velocity commands"""
        velocity_commands = Float64MultiArray()
        velocity_commands.data = []

        for i in range(len(self.joint_names)):
            # Position error
            error = self.desired_positions[i] - self.current_positions[i]

            # Velocity error (desired velocity is 0 for position control)
            velocity_error = 0.0 - self.current_velocities[i]

            # PD control law
            velocity_cmd = self.kp * error + self.kd * velocity_error

            # Saturate velocity command
            max_velocity = 5.0  # rad/s
            velocity_cmd = max(min(velocity_cmd, max_velocity), -max_velocity)

            velocity_commands.data.append(velocity_cmd)

        self.velocity_pub.publish(velocity_commands)


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
