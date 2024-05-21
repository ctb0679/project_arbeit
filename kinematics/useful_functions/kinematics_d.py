#!/usr/bin/env python3

import numpy as np

a = np.array([0, -110.4, -96, 0, 0, 0])
d = np.array([131.22, 0, 0, 63.4, 75.05, 45.6])
alpha = np.pi/2*np.array([1, 0, 0, 1, -1, 0])

#build transformation matrix (j-1)Tj, which describes frame j in frame j-1
def build_singlestep_iTj(i,theta_i):

    T = np.array([[np.cos(theta_i), -np.sin(theta_i)*np.cos(alpha[i]), np.sin(theta_i)*np.sin(alpha[i]), a[i]*np.cos(theta_i)],\
                    [np.sin(theta_i), np.cos(theta_i)*np.cos(alpha[i]), -np.cos(theta_i)*np.sin(alpha[i]), a[i]*np.sin(theta_i)],\
                    [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],\
                    [0, 0, 0, 1]])
    return T

#build transformation matrix iTj, which describes frame j in frame i
def build_iTj(i,j,theta):
    T = np.eye(4)
    for idx in range(i,j):
        T = T@build_singlestep_iTj(idx, theta[idx-i])
    return T