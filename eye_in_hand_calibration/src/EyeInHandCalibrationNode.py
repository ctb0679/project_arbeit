#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import cv2
import yaml
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class EyeInHandCalibration:
    def __init__(self):
        rospy.init_node('eye_in_hand_calibration')

        self.listener = tf.TransformListener()

        self.fk_matrices = []
        self.image_points = []
        self.object_points = []

        self.samples_needed = 10  # Number of samples required
        self.samples_collected = False

        # Chessboard dimensions
        self.chessboard_size = (8, 6)  # Number of inner corners per chessboard row and column
        self.square_size = 0.025  # Size of a square in meters

        # Load camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_camera_parameters('.ros/camera_info/head_camera.yaml')

        # Initialize CV Bridge
        self.bridge = CvBridge()

    def load_camera_parameters(self, filepath):
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
            self.camera_matrix = np.array(data['camera_matrix']['data']).reshape(3, 3)
            self.dist_coeffs = np.array(data['distortion_coefficients']['data'])

    def capture_image(self):
        msg = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=2)
        try:
            # Convert image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_image(self, image):
        ret, corners = cv2.findChessboardCorners(image, self.chessboard_size, None)

        if ret:
            # Refine corner locations
            corners_refined = cv2.cornerSubPix(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), corners, (11, 11), (-1, -1), 
                                               criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            # Generate object points for the checkerboard
            objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
            objp *= self.square_size

            self.image_points.append(corners_refined)
            self.object_points.append(objp)

            # Draw and display the corners
            cv2.drawChessboardCorners(image, self.chessboard_size, corners_refined, ret)
            cv2.imshow("Checkerboard Corners", image)
            cv2.waitKey(1)

            rospy.loginfo("Captured checkerboard pose")
            self.capture_fk_matrix()
        else:
            rospy.logwarn("Checkerboard not found in the image")

    def capture_fk_matrix(self):
        msg = rospy.wait_for_message('fk_matrix', Float64MultiArray, timeout=2)
        matrix = np.array(msg.data).reshape(4, 4)
        self.fk_matrices.append(matrix)
        rospy.loginfo("Captured FK matrix")

    def calibrate(self):
        if len(self.fk_matrices) < self.samples_needed or len(self.image_points) < self.samples_needed:
            rospy.logwarn("Not enough samples collected for calibration")
            return

        # Calibrate camera to find the extrinsic parameters
        ret, _, _, rvecs, tvecs = cv2.calibrateCamera(self.object_points, self.image_points, 
                                                (self.camera_matrix.shape[1], self.camera_matrix.shape[0]), 
                                                self.camera_matrix, self.dist_coeffs)

        R_gripper2base = [m[:3, :3] for m in self.fk_matrices]
        t_gripper2base = [m[:3, 3] for m in self.fk_matrices]
        R_target2cam = [cv2.Rodrigues(r)[0] for r in rvecs]
        t_target2cam = [t.reshape(-1, 3) for t in tvecs]

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
        np.savetxt('cam2gripper_tf.txt', T_cam2gripper.T)
        rospy.loginfo("Eye-in-hand calibration completed and transform saved.")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and len(self.fk_matrices) < self.samples_needed:
            user_input = input("Capture FK matrix and checkerboard pose? (y/n): ")
            if user_input.lower() == "y":
                self.capture_image()
            if len(self.fk_matrices) >= self.samples_needed:
                self.samples_collected = True
                break
            rate.sleep()

        if self.samples_collected:
            self.calibrate()
            rospy.loginfo("Calibration completed. No further poses are being collected.")

if __name__ == '__main__':
    calibration = EyeInHandCalibration()
    calibration.run()
