#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.optimize import minimize
from mycobot_communication.msg import MycobotAngles, MycobotSetAngles
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *


limits = [[-np.radians(165), np.radians(165)], [-np.radians(165), np.radians(165)], [-np.radians(165), np.radians(165)], 
          [-np.radians(165), np.radians(165)], [-np.radians(165), np.radians(165)], [-np.radians(175), np.radians(175)]]

joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
adjusted_limits = [(limit[0] + offset, limit[1] + offset) for limit, offset in zip(limits, joint_offsets)]


def target_function(angles, target_pose):
    """
    Cost function for the optimizer to minimize.
    angles: Current guess of joint angles.
    target_pose: 4x4 numpy array representing the desired end-effector pose.
    Returns: Cost value.
    """
    current_pose = T_lamb(*(angles))
    position_error = np.sum((current_pose[:3, 3] - target_pose[:3, 3])**2)
    orientation_error = np.sum((current_pose[:3, :3] - target_pose[:3, :3])**2)

    return position_error + 0.1 * orientation_error


def inverse_kinematics_minimize(target_pose, initial_guess):
    """
    Solves the inverse kinematics using a minimization approach.
    target_pose: 4x4 numpy array representing the desired end-effector pose.
    Returns: List of joint angles [theta1, theta2, theta3, theta4, theta5, theta6]
    """
    bounds = adjusted_limits
    
    while True:

        result = minimize(target_function, initial_guess, args=(target_pose,), method='L-BFGS-B', bounds=bounds)
        
        if not result.success:
            rospy.logerr("Minimization did not converge: " + result.message)
            continue

        #print(result.x)
        # Ensure the result is within bounds
        for i, angle in enumerate(result.x):
            if angle < bounds[i][0] or angle > bounds[i][1]:
                rospy.logerr(f"Angle {i+1} out of bounds: {angle} not in {bounds[i]}")
                continue
            else:
                return result.x   


def calculate_ik():

    pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=10)
    joint_angles = MycobotSetAngles()
    initial_positions = [0, 0, 0, 0, 0, 0]

    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/mycobot/angles_real', MycobotAngles, timeout=5) 
    
    initial_positions[0] = np.radians(initial_msg.joint_1)
    initial_positions[1] = np.radians(initial_msg.joint_2)
    initial_positions[2] = np.radians(initial_msg.joint_3)
    initial_positions[3] = np.radians(initial_msg.joint_4)
    initial_positions[4] = np.radians(initial_msg.joint_5)
    initial_positions[5] = np.radians(initial_msg.joint_6)

    for i in range(6):
        initial_positions[i] += joint_offsets[i]

    target_point = [0.12, -0.04, 0.37] #In meters
    target_orientation = [1.3, -2.1, 3.0] #In radians
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)

    print("Calculating joint states.....")

    target_js = inverse_kinematics_minimize(np.array(target_pose_matrix), initial_positions) - joint_offsets

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
