#!/usr/bin/env python3

import rospy
from mycobot_communication.msg import MycobotAngles

def real_angles_callback(msg):

    print(msg)
    print()


def read_angles_real():
    rospy.init_node('real_angles_subscriber')
    rospy.Subscriber('/mycobot/angles_real', MycobotAngles, real_angles_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        read_angles_real()
    except rospy.ROSInterruptException:
        pass
