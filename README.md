# Week 4A 

# Q1) q1_forward_kinematics.py: 

This node calculates the 2D end-effector position of a planar 2-link robotic arm using forward kinematics.

It subscribes to the /joint_states topic and extracts the joint angles:

shoulder_pitch_joint = theta1 
elbow_joint = theta2

Adjusts theta1 by adding π/2, as per problem instructions.

Computes the end-effector coordinates (x, y) using:

x=L1cos⁡(theta1)+L2cos⁡(theta1+theta2)

y=L1​sin(theta1​)+L2​sin(theta1​+theta2​)

Publishes the result on the /end_effector_position topic using the message type geometry_msgs/msg/Point.

Logs the position in the terminal.

# Q3) q3_inverse_kinematics.py:

This node performs inverse kinematics for a 2-link planar robotic arm, allowing a user to command the end-effector to move in the x or y direction via terminal input.

Subscribes to /end_effector_position (of type geometry_msgs/msg/Point) to get the current (x, y) position of the end-effector.

Prompts user input via terminal:

Direction to move: 'x' or 'y'

Distance to move (up to 0.5 meters)

Computes a new target position based on the input.

Checks reachability:

Ensures the target lies within the arm's reachable space.

Calculates inverse kinematics:

Computes theta1 and theta2 (joint angles) required to reach the new position.

Adjusts theta1 by subtracting π/2 (to match forward kinematics reference).

Publishes the joint angles as a Float64MultiArray on /joint_angles_goal.

# Bonus Q) bonus.py: 

This node performs 3D forward kinematics for a robotic arm:

base_yaw_joint (theta0): rotation about the Y-axis (upwards in rviz)

shoulder_pitch_joint (theta1): rotation about the Z-axis 

elbow_joint (theta2): rotation about the Z-axis

Constructs transformation matrices using:

Ry(theta0) – yaw at base

Rz(theta1) – shoulder pitch

Tx(L1) – translation along x after shoulder

Rz(theta2) – elbow pitch

Tx(L2) – translation along x after elbow

T = Ry(theta0) @ Rz(theta1) @ Tx(L1) @ Rz(theta2) @ Tx(L2) (@ for multiplication of matrices)

Performs matrix multiplication to compute the final transformation matrix T.

Extracts end-effector position (x, y, z) from the matrix T.

Publishes this position as a geometry_msgs/msg/Point to the topic /end_effector_position.

Logs the position to the terminal.
