# Week 4A 

This ROS 2 Python node calculates the 2D end-effector position of a planar 2-link robotic arm using forward kinematics.

It subscribes to the /joint_states topic and extracts the joint angles:

    shoulder_pitch_joint (θ₁)

    elbow_joint (θ₂)

Adjusts θ₁ by adding π/2, as per problem instructions.

Computes the end-effector coordinates (x, y) using:
x=L1cos⁡(theta1)+L2cos⁡(theta1+theta2)

y=L1​sin(theta1​)+L2​sin(theta1​+theta2​)

Publishes the result on the /end_effector_position topic using the message type geometry_msgs/msg/Point.

Logs the position in the terminal.
