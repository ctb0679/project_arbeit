#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_communication.msg import MycobotSetAngles, MycobotAngles
from math import radians


pub = rospy.Publisher('/joint_states', JointState, queue_size=100)


def publish_js_callback(msg):

    joint_state = JointState()
    joint_state.header = Header()
    sequence = 0
    pos_angles = [0,0,0,0,0,0]

    pos_angles[0] = radians(msg.joint_1)
    pos_angles[1] = radians(msg.joint_2)
    pos_angles[2] = radians(msg.joint_3)
    pos_angles[3] = radians(msg.joint_4)
    pos_angles[4] = radians(msg.joint_5)
    pos_angles[5] = radians(msg.joint_6)

    # JointState message
    joint_state.header.seq = sequence
    joint_state.header.stamp = rospy.Time.now()
    joint_state.header.frame_id = ''
    
    joint_state.name = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']  # Names of the joints
    joint_state.position = pos_angles 
    joint_state.velocity = []
    joint_state.effort = []
    
    # Publish the JointState message
    pub.publish(joint_state)
    
    sequence += 1

if __name__ == '__main__':
    try:
        rospy.init_node('js_publisher')
        rospy.Subscriber('/mycobot/angles_real', MycobotAngles, publish_js_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
