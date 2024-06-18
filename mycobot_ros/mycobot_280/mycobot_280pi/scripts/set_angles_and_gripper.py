#!/usr/bin/env python3

import rospy
import random
from mycobot_communication.msg import MycobotSetAngles, MycobotGripperStatus

def publish_joint_states():

    pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=1)
    pub_gripper = rospy.Publisher('/mycobot/gripper_status', MycobotGripperStatus, queue_size=1)
    rate = rospy.Rate(0.25)

    joint_angles = MycobotSetAngles()
    gripper = MycobotGripperStatus()
    gripper.Status = True

    while not rospy.is_shutdown():
       
        joint_angles.joint_1 = random.uniform(0.1, 50.0)
        joint_angles.joint_2 = random.uniform(0.1, 50.0)
        joint_angles.joint_3 = random.uniform(0.1, 50.0)
        joint_angles.joint_4 = random.uniform(0.1, 50.0)
        joint_angles.joint_5 = random.uniform(0.1, 50.0)
        joint_angles.joint_6 = random.uniform(0.1, 50.0)
        joint_angles.speed = 50

        gripper.Status = not (gripper.Status)
        
        pub.publish(joint_angles)
        pub_gripper.publish(gripper)

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_angles_publisher')
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
