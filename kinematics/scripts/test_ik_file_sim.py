#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from inverse_kinematics import incremental_ik
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *


def calculate_ik():

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(70)

    joint_state = JointState()
    joint_state.header = Header()
    sequence = 0

    joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = initial_msg.position

    target_point = [0.2, -0.06, 0.23] #In metres
    target_orientation = [1.3, -2.1, 3.0] #In degrees
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)

    print("Calculating joint states.....")

    target_js = incremental_ik(initial_positions, np.array(target_pose_matrix))

    for i in range(len(target_js)):
        target_js[i] -= joint_offsets[i]

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
        print("Joint states:", target_js)
        
        sequence += 1
        rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        calculate_ik()
    except rospy.ROSInterruptException:
        pass
