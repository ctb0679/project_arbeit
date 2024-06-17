#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from inverse_kinematics import incremental_ik
from pose_and_matrix_tf import calculate_transformation_matrix
from symb_mat_read import *

# Define the number of steps for interpolation
NUM_STEPS = 100

def calculate_ik():

    pub = rospy.Publisher('/inverse_ik_js', JointState, queue_size=10)
    rate = rospy.Rate(10)

    joint_state = JointState()
    joint_state.header = Header()
    sequence = 0
    
    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = initial_msg.position
    initial_positions = [0, 0, 0, 0, 0, 0]
    target_point = [95, 80, 295] 
    target_orientation = [30, 70, 90] #In degrees
    target_pose_matrix = calculate_transformation_matrix(target_point, target_orientation)

    print("Calculating joint states.....")

    target_js = incremental_ik(initial_positions, np.array(target_pose_matrix))
    joint_state.position = target_js

    print(target_js)    
    print()

    pub.publish(joint_state)
    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        #rospy.Subscriber('/joint_states', JointState, calculate_ik)
        calculate_ik()
    except rospy.ROSInterruptException:
        pass
