#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class EyeInHandCalibration:
    def __init__(self):
        rospy.init_node('eye_in_hand_calibration')

        self.listener = tf.TransformListener()

        self.fk_matrices = []
        self.apriltag_poses = []

        self.samples_needed = 7  # Number of samples required
        self.samples_collected = False

    def capture_fk_matrix(self):
        msg = rospy.wait_for_message('fk_matrix', Float64MultiArray)
        matrix = np.array(msg.data).reshape(4, 4)
        self.fk_matrices.append(matrix)
        rospy.loginfo("Captured FK matrix")

    def capture_apriltag_pose(self):
        msg = rospy.wait_for_message('tf', TFMessage)
        for transform in msg.transforms:
            if transform.child_frame_id == "tag_0":  # Replace with the actual child frame ID of your AprilTag
                matrix = self.transform_to_matrix(transform.transform)
                self.apriltag_poses.append(matrix)
                rospy.loginfo("Captured AprilTag pose")

    def transform_to_matrix(self, transform):
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        matrix = quaternion_matrix(rotation)
        matrix[0:3, 3] = translation
        return matrix

    def calibrate(self):
        if len(self.fk_matrices) < self.samples_needed or len(self.apriltag_poses) < self.samples_needed:
            rospy.logwarn("Not enough samples collected for calibration")
            return

        # Extract rotations and translations
        R_gripper2base = [m[:3, :3] for m in self.fk_matrices]
        t_gripper2base = [m[:3, 3] for m in self.fk_matrices]
        R_target2cam = [m[:3, :3] for m in self.apriltag_poses]
        t_target2cam = [m[:3, 3] for m in self.apriltag_poses]

        
        # Convert to numpy arrays
        R_gripper2base = np.array(R_gripper2base)
        t_gripper2base = np.array(t_gripper2base).reshape(-1, 3)
        R_target2cam = np.array(R_target2cam)
        t_target2cam = np.array(t_target2cam).reshape(-1, 3)

        # Perform eye in hand calibration
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI
        )

        t_cam2gripper = np.squeeze(t_cam2gripper)


        # Create transformation matrix
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        # Save the transformation matrix to a file
        np.savetxt('cam2gripper_tf.txt', T_cam2gripper)
        rospy.loginfo("Eye-in-hand calibration completed and transform saved.")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and len(self.fk_matrices) < self.samples_needed:
            user_input = input("Capture AprilTag pose? (y/n): ")
            if user_input.lower() == "y":
                self.capture_fk_matrix()
                self.capture_apriltag_pose()
            if len(self.fk_matrices) >= self.samples_needed and len(self.apriltag_poses) >= self.samples_needed:
                self.samples_collected = True
                break
            rate.sleep()

        if self.samples_collected:
            self.calibrate()
            rospy.loginfo("Calibration completed. No further poses are being collected.")

if __name__ == '__main__':
    calibration = EyeInHandCalibration()
    calibration.run()
