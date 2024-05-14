#!/usr/bin/env python3

from symb_mat_read import T_lamb
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


pub = rospy.Publisher('fk_matrix', Float64MultiArray, queue_size=10)

def js_callback(data):

    rate = rospy.Rate(10)

    joint_angles = np.around(data.position, decimals=5)
    # print(joint_angles)

    joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

    for i in range(len(joint_angles)):
        joint_angles[i] += joint_offsets[i]
    
    trans_mat_final = np.around(T_lamb(*(joint_angles)), decimals=5)

    final_tf = Float64MultiArray()
    final_tf.data = trans_mat_final.flatten().tolist()

    rospy.loginfo(final_tf)
    pub.publish(final_tf)
    rate.sleep()


def listener():
    rospy.init_node("FK_node", anonymous=True)
    rospy.Subscriber("joint_states", JointState, js_callback, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    listener()