#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class Forward_Kinematics_Node(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Link lengths
        self.L1 = 2.0  # meters
        self.L2 = 1.5  # meters

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publish to end effector position
        self.publisher = self.create_publisher(Point, '/end_effector_position', 10)

        self.get_logger().info('Forward Kinematics Node Initialized')


    def joint_callback(self, msg):
        # Extract theta and theta2 from the JointState message
        theta1 = msg.position[msg.name.index('shoulder_pitch_joint')]
        theta2 = msg.position[msg.name.index('elbow_joint')]

        # Adjust theta1 reference as per instruction
        theta1 += math.pi / 2

        # Forward kinematics equations
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)

        # Publish the position
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0  # Unused in 2D case

        self.publisher.publish(point)
        self.get_logger().info(f'Published End Effector Position: x={x:.2f}, y={y:.2f}')
    


def main(args=None):
    rclpy.init(args=args)
    node = Forward_Kinematics_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
