#!/usr/bin/env python3

from symb_mat_read import T_lamb
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from pose_and_matrix_tf import extract_translation_and_rotation


pub = rospy.Publisher('fk_matrix', Float64MultiArray, queue_size=10)

def js_callback(data):

    rate = rospy.Rate(70)

    # joint_angles = np.around(data.position, decimals=5)

    joint_angles = [0, -np.pi/2, 0, np.pi/2, 0, 0]
    # joint_angles = [0.0, 1.0472, 1.5708, -2.61799, 3.14, 0.0]
    # joint_angles = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]
    joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

    for i in range(len(joint_angles)):
        joint_angles[i] += joint_offsets[i]
    
    trans_mat_final = np.around(T_lamb(*(joint_angles)), decimals=5)

    final_tf = Float64MultiArray()
    final_tf.data = trans_mat_final.flatten().tolist()

    pose_vec = extract_translation_and_rotation(trans_mat_final)
    print(pose_vec)

    #rospy.loginfo(final_tf)
    pub.publish(final_tf)
    rate.sleep()


def listener():
    rospy.init_node("FK_node", anonymous=True)
    rospy.Subscriber("joint_states", JointState, js_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()