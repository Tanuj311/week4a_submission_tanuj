#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math

class Inverse_Kinematics_Node(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        self.L1 = 2.0
        self.L2 = 1.5
        self.current_x = None
        self.current_y = None

        self.create_subscription(Point, '/end_effector_position', self.position_callback, 10)

        # Publish the joint angles
        self.joint_publish = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)

        self.get_logger().info('Inverse Kinematics Node Initialized. Waiting for position..')

        # Timer to allow user input
        self.create_timer(1.0, self.user_input_loop)

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y

    def user_input_loop(self):
        if self.current_x is None or self.current_y is None:
            return  # Wait until position is received
    
        # Taking input 
        direction = input("Enter direction to move (x or y): ").strip().lower()
        if direction != 'x' and direction != 'y':
            self.get_logger().warn("Invalid direction. Use 'x' or 'y'.")
            return

        # Checking max distance limit
        distance = float(input("Enter distance to move (max 0.5 m): "))
        if abs(distance) > 0.5:
            self.get_logger().warn("Distance must be ≤ 0.5 meters.")
            return

        # Calculate new position
        target_x = self.current_x + distance if direction == 'x' else self.current_x
        target_y = self.current_y + distance if direction == 'y' else self.current_y

        # Check reachability
        d_squared = target_x**2 + target_y**2
        if d_squared > (self.L1 + self.L2)**2 or d_squared < (self.L1 - self.L2)**2:
            self.get_logger().warn("Target position is unreachable.")
            return

        # Inverse Kinematics
        theta2 = math.acos((target_x**2 + target_y**2 - self.L1**2 - self.L2**2)/(2*self.L1*self.L2))
        theta1 = math.atan2(target_y, target_x) - math.atan2(self.L2*math.sin(theta2), self.L1 + self.L2*math.cos(theta2))
        
        theta1 -= math.pi / 2

        # Publish joint angles
        msg = Float64MultiArray()
        msg.data = [theta1, theta2]
        self.joint_publish.publish(msg)

        self.get_logger().info(
            f"Published Joint Angles: theta1 = {theta1:.2f} rad, theta2 = {theta2:.2f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Inverse_Kinematics_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
