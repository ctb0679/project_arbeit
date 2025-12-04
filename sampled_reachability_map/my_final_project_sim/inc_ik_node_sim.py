#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from incremental_ik import incremental_ik
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *


def calculate_ik():

    pub = rospy.Publisher('/ik_node', JointState, queue_size=10)
    rate = rospy.Rate(70)

    joint_state = JointState()
    joint_state.header = Header()
    sequence = 0

    #joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = initial_msg.position
    initial_positions = [0, 0, 0, 0, 0, 0]
    final_angles = [0, 0, 0, 0, 0, 0]

    target_point = [-0.1488, -0.06333, 0.21874] #In meters
    target_orientation = [-1.57, 0, 1.57] #In radians
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)
    print(target_pose_matrix)
    print("Calculating joint states.....")

    target_js = incremental_ik(initial_positions, np.array(target_pose_matrix))

    for i in range(len(target_js)):
        final_angles[i] = np.degrees(target_js[i])

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

        print(target_js)
        print("Joint states:", final_angles)
        
        sequence += 1
        rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        calculate_ik()
    except rospy.ROSInterruptException:
        pass
