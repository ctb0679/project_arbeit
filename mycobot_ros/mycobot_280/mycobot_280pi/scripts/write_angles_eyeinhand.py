#!/usr/bin/env python3

import rospy
import random
from mycobot_communication.msg import MycobotSetAngles

def publish_joint_states():

    pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=1)
    rate = rospy.Rate(0.07)

    joint_angles = MycobotSetAngles()

    js_list = [[0.0, 50.0, 0.0, -30.0, 5.0, -70.0], [60.0, -55.0, -30.0, -30.0, -35.0, -75.0], [95.0, -25.0, -30.0, -70.0, -35.0, -75.0], [0.0, -75.0, 20.0, -44.0, 15.0, 35.0],
                [-30.0, -75.0, 20.0, -41.0, 40.0, -145.0], [-65.0, -37.0, -20.0, -41.0, 42.0, -145.0], [-89.0, -41.0, -20.0, -49.0, 42.0, -145.0], [-45.0, -61.0, 15.0, -49.0, 42.0, -145.0],
                [34.0, 20.0, 15.0, -89.0, -15.0, -45.0], [7.0, 33.0, -55.0, -39.0, 11.0, 97.0]]
    

    while not rospy.is_shutdown():

        i = 0

        for i in range(10):     
            joint_angles.joint_1 = js_list[i][0]
            joint_angles.joint_2 = js_list[i][1]
            joint_angles.joint_3 = js_list[i][2]
            joint_angles.joint_4 = js_list[i][3]
            joint_angles.joint_5 = js_list[i][4]
            joint_angles.joint_6 = js_list[i][5]
            joint_angles.speed = 50
            
            pub.publish(joint_angles)

            rate.sleep()
            i += 1

        break

if __name__ == '__main__':
    try:
        rospy.init_node('joint_angles_publisher')
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
