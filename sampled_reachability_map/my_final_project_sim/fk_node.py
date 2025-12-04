#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from kinematics.scripts.pose_and_matrix_tf import extract_translation_and_rotation
from kinematics.scripts.symb_mat_read import T_lamb  # FK function that computes T_base_to_ee

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
    """
    Callback function that computes FK, extracts Z_INIT, and publishes FK matrix.
    :param data: JointState message containing joint positions
    """
    joint_angles = np.array(data.position)  # Extract joint angles

    # Apply joint offsets
    joint_offsets = np.array([0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0])
    joint_angles += joint_offsets  # Adjust angles

    # Compute Forward Kinematics (FK) transformation matrix
    trans_mat_final = np.around(T_lamb(*joint_angles), decimals=5)  # 4x4 matrix

    # Compute the correct Z_INIT
    Z_INIT_corrected = compute_z_init(trans_mat_final)
    rospy.set_param('/Z_INIT', Z_INIT_corrected)  # Store in ROS parameter server
    rospy.loginfo(f"Computed Z_INIT: {Z_INIT_corrected:.4f} meters")

    # Publish FK matrix as a flattened list
    final_tf = Float64MultiArray()
    final_tf.data = trans_mat_final.flatten().tolist()
    pub.publish(final_tf)

def listener():
    """ Initializes the ROS node and subscribes to joint states """
    rospy.init_node("FK_node", anonymous=True)
    rospy.Subscriber("joint_states", JointState, js_callback)
    rospy.spin()  # Keep running

if __name__ == '__main__':
    listener()
