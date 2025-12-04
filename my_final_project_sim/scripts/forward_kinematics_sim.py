#!/usr/bin/env python3

from symb_mat_read import T_lamb
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from pose_and_matrix_tf import extract_translation_and_rotation

# ROS Publisher
pub = rospy.Publisher('fk_matrix', Float64MultiArray, queue_size=10)

def compute_z_init(fk_matrix):
    """
    Computes the correct Z_INIT by projecting the end-effector position onto its Z-axis.
    :param fk_matrix: 4x4 Transformation matrix from base to end-effector
    :return: Corrected Z_INIT in the base frame
    """
    translation, rotation_matrix = extract_translation_and_rotation(fk_matrix)
    
    # Extract the end-effector Z-axis direction in the base frame
    z_axis_ee_in_base = rotation_matrix[:, 2]  # Third column of the rotation matrix

    # Project translation onto the end-effector's Z direction
    Z_INIT_corrected = np.dot(translation, z_axis_ee_in_base)

    return Z_INIT_corrected

def js_callback(data):

    rate = rospy.Rate(70)

    joint_angles = np.around(data.position, decimals=5)

    joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

    for i in range(len(joint_angles)):
        joint_angles[i] += joint_offsets[i]
    
    trans_mat_final = np.around(T_lamb(*(joint_angles)), decimals=5)

    # Compute the correct Z_INIT
    Z_INIT_corrected = compute_z_init(trans_mat_final)
    rospy.set_param('/Z_INIT', Z_INIT_corrected)  # Store in ROS parameter server
    rospy.loginfo(f"Computed Z_INIT: {Z_INIT_corrected:.4f} meters")

    final_tf = Float64MultiArray()
    final_tf.data = trans_mat_final.flatten().tolist()

    #rospy.loginfo(final_tf)
    pub.publish(final_tf)
    rate.sleep()


def listener():
    rospy.init_node("FK_node", anonymous=True)
    rospy.Subscriber("joint_states", JointState, js_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()