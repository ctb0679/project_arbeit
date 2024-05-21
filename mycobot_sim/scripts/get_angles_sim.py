#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    joint_names = msg.name
    joint_positions = msg.position

    print(joint_names, ": ", joint_positions)
    print()


def subscribe_to_joint_states():
    rospy.init_node('joint_state_subscriber')
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_to_joint_states()
    except rospy.ROSInterruptException:
        pass
