#!/usr/bin/env python3

import sympy as sp
from sympy import lambdify


theta1,theta2,theta3,theta4,theta5,theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')


with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/eye_in_hand_calibration/our_useful_functions/symb_jacobian.txt", "r") as inf:
    J = sp.Matrix(sp.sympify(inf.read()))

with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/eye_in_hand_calibration/our_useful_functions/symb_transform.txt", "r") as inf:
    A = sp.Matrix(sp.sympify(inf.read()))

with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/eye_in_hand_calibration/our_useful_functions/symb_transform_matrix.txt", "r") as inf:
    T = sp.Matrix(sp.sympify(inf.read()))

A_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), A, 'numpy'))
J_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), J, 'numpy'))
T_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), T, 'numpy'))
