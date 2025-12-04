#!/usr/bin/env python
import rospy
import numpy as np
from mycobot_communication.msg import MycobotSetAngles
from kinematics.scripts.ik_node import inverse_kinematics_minimize  # Your IK solver

# Fixed orientation
FIXED_ORIENTATION = (0, 1.57, 0)
Z_INIT = 0.1  # Set based on wall distance

# Trajectory points (x, y) relative to (0,0) start position
trajectory_points = [(0.02, 0.05), (0.03, 0.10), (0.04, 0.15), (0.05, 0.20)]

def compute_full_pose(xy_points):
    """ Convert (x, y) to full 6D pose (x, y, z, roll, pitch, yaw) """
    return [(x, y, Z_INIT, *FIXED_ORIENTATION) for x, y in xy_points]

def execute_trajectory():
    rospy.init_node('cobot_trajectory', anonymous=True)
    pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=10)
    rate = rospy.Rate(10)

    # Convert to full 6D poses
    full_poses = compute_full_pose(trajectory_points)

    for pose in full_poses:
        x, y, z, roll, pitch, yaw = pose
        joint_angles = inverse_kinematics_minimize((x, y, z, roll, pitch, yaw))

        if joint_angles is not None:
            joint_msg = MycobotSetAngles()
            joint_msg.joint_1, joint_msg.joint_2, joint_msg.joint_3 = joint_angles[:3]
            joint_msg.joint_4, joint_msg.joint_5, joint_msg.joint_6 = joint_angles[3:]
            joint_msg.speed = 50  # Adjust speed as needed

            pub.publish(joint_msg)
            rospy.sleep(0.5)  # Wait for movement

if __name__ == '__main__':
    try:
        execute_trajectory()
    except rospy.ROSInterruptException:
        pass
