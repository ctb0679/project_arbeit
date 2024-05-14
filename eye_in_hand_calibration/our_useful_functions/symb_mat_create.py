#!/usr/bin/env python3

import sympy as sp
import numpy as np
from kinematics_d import a,d,alpha

#build symbolic transformation matrix (j-1)Tj, which describes frame j in frame j-1
def build_singlestep_iTj_symb(i,theta_i):

    T = sp.Matrix([[sp.cos(theta_i), -sp.sin(theta_i)*sp.cos(alpha[i]), sp.sin(theta_i)*sp.sin(alpha[i]), a[i]*sp.cos(theta_i)],\
                    [sp.sin(theta_i), sp.cos(theta_i)*sp.cos(alpha[i]), -sp.cos(theta_i)*sp.sin(alpha[i]), a[i]*sp.sin(theta_i)],\
                    [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],\
                    [0, 0, 0, 1]])
    return T

#build symbolic transformation matrix iTj, which describes frame j in frame i
def build_iTj_symb(i,j,theta):
    T = sp.eye(4)
    for idx in range(i,j):
        T = sp.trigsimp(sp.nsimplify(T@build_singlestep_iTj_symb(idx, theta[idx-i]),tolerance=1e-8))
        print(idx)
    return T


theta1,theta2,theta3,theta4,theta5,theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
# theta = [0,0,0,0,np.pi,0,0]

T_0_6_symb = build_iTj_symb(0,6,[theta1,theta2,theta3,theta4,theta5,theta6])
# print(T_0_6_symb)


A = T_0_6_symb[0:3, 0:4] # crop last row
A = A.transpose().reshape(12,1) #reshape to column vector

J = A.jacobian(sp.Matrix([theta1,theta2,theta3,theta4,theta5,theta6]))
J = sp.trigsimp(sp.nsimplify(J,tolerance=1e-8))
# print(J,J.shape)

with open("symb_jacobian.txt", "w") as outf:
    outf.write(str(J))

with open("symb_transform.txt", "w") as outf:
    outf.write(str(A))

with open("symb_transform_matrix.txt", "w") as outf:
    outf.write(str(T_0_6_symb))