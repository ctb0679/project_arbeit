import random
import numpy as np
from symb_mat_read import J_lamb, A_lamb

# Define the robot joint limits (in radians)
limits = [
    [-np.radians(165), np.radians(165)],  # Joint 1
    [-np.radians(165), np.radians(165)],  # Joint 2
    [-np.radians(165), np.radians(165)],  # Joint 3
    [-np.radians(165), np.radians(165)],  # Joint 4
    [-np.radians(165), np.radians(165)],  # Joint 5
    [-np.radians(175), np.radians(175)]   # Joint 6
]

# Function to check if the joint vector is within the specified limits
def check_joint_vector_within_limits(joint_vector, limits):
    for i in range(len(joint_vector)):
        if joint_vector[i] < limits[i][0]:
            while joint_vector[i] < limits[i][0]:
                joint_vector[i] += 2 * np.pi
        elif joint_vector[i] > limits[i][1]:
            while joint_vector[i] > limits[i][1]:
                joint_vector[i] -= 2 * np.pi

        if not (limits[i][0] <= joint_vector[i] <= limits[i][1]):
            return False

    return True

# Function to create a random starting point for IK if the provided one doesn't work
def generate_random_vector():
    random_vector = []
    for limit in limits:
        lower_limit, upper_limit = limit
        random_angle = random.uniform(lower_limit, upper_limit)
        random_vector.append(random_angle)
    return random_vector

# Define a small threshold for singularity detection
SINGULARITY_THRESHOLD = 1e-6

# Function to check for singularity using the determinant of the Jacobian
def is_singular(joint_angles):
    J_current = J_lamb(*joint_angles)
    rank_J = np.linalg.matrix_rank(J_current)
    if rank_J < min(J_current.shape):
        print("Singularity detected. Skipping this IK solution.")
        return True
    return False

# Exponential moving average to smooth the joint angles
def smooth_joint_angles(prev_joint_angles, new_joint_angles, alpha=0.1):
    return alpha * np.array(new_joint_angles) + (1 - alpha) * np.array(prev_joint_angles)

# Main IK function
def incremental_ik(init_joint_state, goal, steps=1000, tol=0.001):
    q_current = np.array(init_joint_state)

    while True:
        # Calculate current pose
        current_pose = A_lamb(*q_current)

        # Get the goal pose in (12,1) vector
        goal_pose = goal[0:3, 0:4].T.reshape(12, 1)  # Reshape the goal pose
        error = goal_pose - current_pose
        i = 0

        while np.max(np.abs(error)) > tol and i < steps:
            J_current = J_lamb(*q_current)  # Get current Jacobian
            if is_singular(q_current):  # Check for singularity
                print("Singularity detected. Skipping this IK solution.")
                return None  # Skip the solution if singularity detected

            delta_q = np.linalg.pinv(J_current) @ error
            q_current += delta_q.flatten()

            current_pose = A_lamb(*q_current)  # Update current pose
            error = goal_pose - current_pose
            i += 1

        # Apply smoothing
        q_current = smooth_joint_angles(init_joint_state, q_current, alpha=0.1)

        # Check if the solution is within limits
        limit_check = check_joint_vector_within_limits(q_current, limits)
        if limit_check:
            break
        else:
            q_current = generate_random_vector()

    return q_current
