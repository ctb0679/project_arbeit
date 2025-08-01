#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *


limits = [(-np.radians(165), np.radians(165)), (-np.radians(165), np.radians(165)), (-np.radians(165), np.radians(165)), 
          (-np.radians(165), np.radians(165)), (-np.radians(165), np.radians(165)), (-np.radians(175), np.radians(175))]

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

    pub = rospy.Publisher('/ik_node', JointState, queue_size=10)
    fnl_st = [0, 0, 0, 0, 0, 0]
    rate = rospy.Rate(70)

    joint_state = JointState()
    joint_state.header = Header()
    initial_positions = [0, 0, 0, 0, 0, 0]
    sequence = 0

    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    
    for i in range(6):
        initial_positions[i] = initial_msg.position[i] + joint_offsets[i]

    #initial_positions = [0, 0, 0, 0, 0, 0]

    target_point = [0.04, -0.06, 0.412] #In meters
    target_orientation = [1.3, -1.1, 2.0] #In radians
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)

    print("Calculating joint states.....")

    target_js = inverse_kinematics_minimize(np.array(target_pose_matrix), initial_positions) - joint_offsets

    for i in range(len(target_js)):
        fnl_st[i] = np.rad2deg(target_js[i])

    while not rospy.is_shutdown():
        joint_state.header.seq = sequence
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = ''
        
        joint_state.name = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'] 
        joint_state.position = target_js 
        joint_state.velocity = []
        joint_state.effort = []
        
        # Publish the JointState message
        pub.publish(joint_state)
        print("Joint states:", fnl_st)
        
        sequence += 1
        rate.sleep()

   

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        calculate_ik()
    except rospy.ROSInterruptException:
        pass
