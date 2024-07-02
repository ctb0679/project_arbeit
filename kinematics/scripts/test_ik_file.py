#!/usr/bin/env python3

import rospy
import numpy as np
from inverse_kinematics import incremental_ik
from mycobot_communication.msg import MycobotAngles, MycobotSetAngles
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *


def calculate_ik():

    pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=10)
    joint_angles = MycobotSetAngles()
    rate = rospy.Rate(10)

    joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
    initial_positions = [0, 0, 0, 0, 0, 0]

    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/mycobot/angles_real', MycobotAngles, timeout=5) 

    initial_positions[0] = initial_msg.joint_1
    initial_positions[1] = initial_msg.joint_2
    initial_positions[2] = initial_msg.joint_3
    initial_positions[3] = initial_msg.joint_4
    initial_positions[4] = initial_msg.joint_5
    initial_positions[5] = initial_msg.joint_6

    target_point = [0.2, -0.06, 0.23] #In metres
    target_orientation = [1.3, -2.1, 3.0] #In radians
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)

    print("Calculating joint states.....")

    target_js = incremental_ik(initial_positions, np.array(target_pose_matrix))

    for i in range(len(target_js)):
        target_js[i] -= joint_offsets[i]

    joint_angles.joint_1 = np.rad2deg(target_js[0])
    joint_angles.joint_2 = np.rad2deg(target_js[1])
    joint_angles.joint_3 = np.rad2deg(target_js[2])
    joint_angles.joint_4 = np.rad2deg(target_js[3])
    joint_angles.joint_5 = np.rad2deg(target_js[4])
    joint_angles.joint_6 = np.rad2deg(target_js[5])
    joint_angles.speed = 50
    
    # Publish the JointState message
    pub.publish(joint_angles)
    rospy.loginfo(joint_angles)        

   

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        calculate_ik()
    except rospy.ROSInterruptException:
        pass
