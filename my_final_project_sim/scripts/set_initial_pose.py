#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

NUM_STEPS = 100  # Number of interpolation steps

def publish_joint_states():
    """ Publishes joint states to move the robot to the initial position smoothly. """

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('set_initial_pose', anonymous=True)
    rate = rospy.Rate(60)

    # Wait for initial joint states
    rospy.loginfo("Waiting for /joint_states...")
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = list(initial_msg.position)

    # Desired initial joint angles (converted from radians)
    desired_positions = [1.5708, 0.523599, 2.0944, -2.61799, 3.14, 0.0]

    rospy.loginfo("Starting interpolation to initial pose...")
    for step in range(NUM_STEPS):
        interpolated_positions = [
            initial_positions[i] + (desired_positions[i] - initial_positions[i]) * (step / NUM_STEPS)
            for i in range(len(initial_positions))
        ]

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        joint_state.position = interpolated_positions
        pub.publish(joint_state)
        rate.sleep()

    rospy.loginfo("Reached initial pose.")

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
