import numpy as np
import cv2 as cv
import sys,os
sys.path.append(os.getcwd())
from our_useful_functions.symb_mat_read import A_lamb

# additional functions
def read_matrix_from_file(filepath):
    return np.loadtxt(filepath)


def get_rotation_matrices_and_translation_vectors(A_collect):
    R = np.zeros([A_collect.shape[0], 3, 3])
    t = np.zeros([A_collect.shape[0], 3])
    for i in range(A_collect.shape[0]):
        Mat_4x4 = A_collect[i, :].reshape([4, 4])
        R[i, :, :] = Mat_4x4[:3, :3]
        t[i, :] = Mat_4x4[:3, 3]
    return R, t


def get_transformation_matrix(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t.reshape(-1)
    return M


def build_R_tensor_from_rot_vec(rot_vec):
    n = rot_vec.shape[1]
    R = np.zeros([n, 3, 3])
    for i in range(n):
        R[i, :, :] = cv.Rodrigues(rot_vec[:, i])[0]
    return R


def build_transformation_from_joint_states(joint_states):
    n = joint_states.shape[1]
    T = np.zeros([n, 3, 4])
    for i in range(n):
        T[i, :, :] = A_lamb(*(joint_states[:, i].flatten())).reshape(
            [3, 4], order="F"
        )  # order is in Fortran mode meaning first index changes the fastest
    return T[:, :, :3], T[:, :, 3]

# test for HandEye Calibration
def test_HE(T_w2c, T_g2b, T_c2g_cal):
    sum_error_diff = 0
    sum_error_mult = 0
    for i in range(len(T_w2c) - 1):
        sum_error_diff += (
            np.linalg.norm(T_g2b[i] @ T_c2g_cal @ T_w2c[i] - T_g2b[i + 1] @ T_c2g_cal @ T_w2c[i + 1])
        )
        sum_error_mult += (
            np.linalg.norm(T_g2b[i] @ T_c2g_cal @ T_w2c[i] @ np.linalg.inv(T_g2b[i + 1] @ T_c2g_cal @ T_w2c[i + 1]) - np.eye(4))
        )
    return sum_error_diff/(len(T_w2c)-1), sum_error_mult/(len(T_w2c)-1)

