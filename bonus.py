#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math
import numpy as np

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
        # Extract theta1 and theta2 from the JointState message
        theta1 = msg.position[msg.name.index('shoulder_pitch_joint')]
        theta2 = msg.position[msg.name.index('elbow_joint')]
        theta0 = msg.position[msg.name.index('base_yaw_joint')]       

        theta1 += math.pi/2

        # Extract x,y,z from transformation matrix
        T = self.forward_kinematics(theta0,theta1,theta2)
        x,y,z = T[0,3] ,T[1,3],T[2,3]

        # Publish the position
        point = Point()
        point.x = x
        point.y = y
        point.z = z  

        self.publisher.publish(point)
        self.get_logger().info(f'Published End Effector Position: x={x:.2f}, y={y:.2f}, z={z:.2f}')

    def forward_kinematics(self, theta0,theta1,theta2):
        def Rz(theta):
            return np.array([
                [np.cos(theta), -np.sin(theta), 0, 0],
                [np.sin(theta),  np.cos(theta), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        def Ry(theta):
            return np.array([
                [np.cos(theta), 0, np.sin(theta), 0],
                [0, 1, 0, 0],
                [-np.sin(theta), 0, np.cos(theta), 0],
                [0, 0, 0, 1]
            ])

        def Tx(l):
            return np.array([
                [1, 0, 0, l],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        
        # Matrix multiplication
        T = Ry(theta0) @ Rz(theta1) @ Tx(self.L1) @ Rz(theta2) @ Tx(self.L2)
        return T

def main(args=None):
    rclpy.init(args=args)
    node = Forward_Kinematics_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
