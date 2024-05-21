#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Define the number of steps for interpolation
NUM_STEPS = 100

def publish_joint_states():

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(60)

    joint_state = JointState()
    joint_state.header = Header()
    sequence = 0
    
    # Define the initial and desired joint angles
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = initial_msg.position
    desired_positions = [1.0, 0.0, 1.7, -1, -2, -2.5]

    while not rospy.is_shutdown():
        # Interpolate joint angles  
        interpolated_positions = []
        for i in range(len(initial_positions)):
            delta = (desired_positions[i] - initial_positions[i]) / NUM_STEPS
            interpolated_positions.append(initial_positions[i] + delta * sequence)

        # JointState message
        joint_state.header.seq = sequence
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = ''
        
        joint_state.name = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']  # Names of the joints
        joint_state.position = interpolated_positions 
        joint_state.velocity = []
        joint_state.effort = []
        
        # Publish the JointState message
        pub.publish(joint_state)
        
        sequence += 1
        if sequence > NUM_STEPS:
            initial_positions = desired_positions  # Update initial positions

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        #rospy.Subscriber('/joint_states', JointState, publish_joint_states)
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
