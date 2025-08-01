import numpy as np
import random
import rospy
import open3d as o3d  # Optional: for visualization (PointCloud)
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.robot_state import RobotState
from symb_mat_read import T_lamb  # Your custom forward kinematics function
from incremental_ik import incremental_ik  # Your custom inverse kinematics solver

# Joint limits for your manipulator (in radians)
limits = [
    [-np.radians(165), np.radians(165)],
    [-np.radians(165), np.radians(165)],
    [-np.radians(165), np.radians(165)],
    [-np.radians(165), np.radians(165)],
    [-np.radians(165), np.radians(165)],
    [-np.radians(175), np.radians(175)]
]

# Function to sample random joint angles within the limits
def sample_joint_vector():
    return np.array([random.uniform(l[0], l[1]) for l in limits])

def compute_fk_pose(joint_angles):
    # Compute the transformation matrix from your FK function (T_lamb)
    pose_matrix = T_lamb(*joint_angles)
    # Extract the position (translation) of the end-effector from the matrix
    position = pose_matrix[:3, 3]  # [x, y, z]
    return position

def generate_reachability_map(num_samples=10000):
    reachable_positions = []
    for _ in range(num_samples):
        joints = sample_joint_vector()  # Sample random joint configuration
        try:
            # Compute FK to get the end-effector position
            pos = compute_fk_pose(joints)
            reachable_positions.append(pos)
        except Exception as e:
            # Handle IK failure if needed
            print(f"Error: {e}")
            continue
    return np.array(reachable_positions)

def save_as_pointcloud(points, filename='reachability_map.pcd'):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, pc)

def save_as_txt(points, filename='reachability_map.txt'):
    np.savetxt(filename, points)
