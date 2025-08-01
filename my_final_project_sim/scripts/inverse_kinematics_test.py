#!/usr/bin/env python3

import numpy as np
from scipy.optimize import minimize

ROTATION = np.array([
    [0.1975983, -0.5631220, -0.8024017],
    [0.5631220, -0.6048034,  0.5631220],
    [-0.8024017, -0.5631220,  0.1975983]
])

# Approximate D-H Parameters for MyCobot 280 Pi
DH_PARAMS = [
    {'d': 0.1104, 'a': 0.0,    'alpha': np.pi/2},
    {'d': 0.0,    'a': 0.096,  'alpha': 0.0},
    {'d': 0.0,    'a': 0.073,  'alpha': 0.0},
    {'d': 0.073,  'a': 0.0,    'alpha': np.pi/2},
    {'d': 0.0,    'a': 0.0,    'alpha': -np.pi/2},
    {'d': 0.0456, 'a': 0.0,    'alpha': 0.0},
]

# Joint limits and offsets
limits = [[-np.radians(165), np.radians(165)],
          [-np.radians(165), np.radians(165)],
          [-np.radians(165), np.radians(165)],
          [-np.radians(165), np.radians(165)],
          [-np.radians(165), np.radians(165)],
          [-np.radians(175), np.radians(175)]]

joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
adjusted_limits = [(limit[0] + offset, limit[1] + offset) for limit, offset in zip(limits, joint_offsets)]

def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i in range(6):
        params = DH_PARAMS[i]
        T = T @ dh_transform(joint_angles[i], params['d'], params['a'], params['alpha'])
    return T

def pose_error(joint_angles, target_pose):
    T_current = forward_kinematics(joint_angles)
    pos_error = np.linalg.norm(T_current[:3, 3] - target_pose[:3, 3])
    rot_error = np.linalg.norm(T_current[:3, :3] - target_pose[:3, :3])
    return pos_error + 0.5 * rot_error

def inverse_kinematics(target_pose, initial_guess=None):
    if initial_guess is None:
        initial_guess = np.zeros(6)

    result = minimize(
        pose_error,
        initial_guess,
        args=(target_pose,),
        method='L-BFGS-B',
        bounds=adjusted_limits
    )

    if result.success:
        return result.x
    else:
        print("IK solver failed:", result.message)
        return None

# Example Usage
if __name__ == "__main__":
    # Example target pose (modify this with actual desired 4x4 matrix)
    target_pose = np.eye(4)
    target_pose[:3, 3] = [-0.1488, -0.0185, 0.0185]  # target position
    target_pose[:3, :3] = ROTATION

    solution = inverse_kinematics(target_pose)
    if solution is not None:
        print("IK Solution (joint angles in radians):", solution)
        print("IK Solution (joint angles in degrees):", np.degrees(solution))
    else:
        print("No valid IK solution found.")
