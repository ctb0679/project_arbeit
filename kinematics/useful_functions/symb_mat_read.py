#!/usr/bin/env python3

import sympy as sp
import numpy as np
from sympy import lambdify
import os
import getpass

theta1,theta2,theta3,theta4,theta5,theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')

if os.getcwd().endswith("y"):  # we are in our repo folder, not in catkin_ws, started via vscode
    additional_dir = ""
elif os.getcwd().endswith("ws"):    # we are in catkin_ws, started via rosrun
    additional_dir = "src/scientific-working-ss-24/"
elif os.getcwd().endswith("os"):    # swe are in /home/rnm/.ros, started via launchfile
    username = getpass.getuser()
    additional_dir = "/home/"+username+"/catkin_ws/src/scientific-working-ss-24/"


with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/kinematics/useful_functions/symb_jacobian.txt", "r") as inf:
    J = sp.Matrix(sp.sympify(inf.read()))

with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/kinematics/useful_functions/symb_transform.txt", "r") as inf:
    A = sp.Matrix(sp.sympify(inf.read()))

with open("/home/ayush15/catkin_ws/src/scientific-working-ss-24/kinematics/useful_functions/symb_transform_matrix.txt", "r") as inf:
    T = sp.Matrix(sp.sympify(inf.read()))

A_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), A, 'numpy'))
J_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), J, 'numpy'))
T_lamb = (lambdify((theta1,theta2,theta3,theta4,theta5,theta6), T, 'numpy'))

#theta = [2,0.333,0,np.pi/37,np.pi,0,0]
#J_eval = np.array(J.evalf(subs={theta1: theta[0],theta2: theta[1],theta3: theta[2],theta4: theta[3],theta5: theta[4],theta6: theta[5],theta7: theta[6]}))
#print(J_eval)