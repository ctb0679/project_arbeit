#!/usr/bin/env python3

import rospy
import numpy as np
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
    # initial_positions = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]
    rospy.loginfo(initial_positions)
    # desired_positions = [2.326991354714245, 0.3913173844007911, 0.8667142257435856, 1.5725109895718088, -2.0277781473196708, 2.3648425211936726]
    # desired_positions = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]
    desired_positions_0 = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]
    desired_positions_1 = [2.52897480e-01,  4.85718905e-01,  1.85347273e+00, -2.34022830e+00, 2.87979327e+00, -2.61922847e-04]
    waypoints = [[0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0],
                 [2.52897480e-01,  4.85718905e-01,  1.85347273e+00, -2.34022830e+00, 2.87979327e+00, -2.61922847e-04]]
    try:
        while not rospy.is_shutdown():
            # Interpolate joint angles  
            for idx, desired_positions in enumerate(waypoints):
                rospy.loginfo(f"Moving to waypoint {idx+1}: {desired_positions}")
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
                    initial_positions = desired_positions # Update initial positions

                rate.sleep()
      
    except KeyboardInterrupt:
        rospy.logwarn("Trajectory execution interrupted. Exiting...")
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS shutdown requested. Exiting...")
    
if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        #rospy.Subscriber('/joint_states', JointState, publish_joint_states)
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
