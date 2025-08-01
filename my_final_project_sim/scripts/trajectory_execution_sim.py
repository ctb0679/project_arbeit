#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ik_node import inverse_kinematics_minimize  # Your IK solver
from pose_and_matrix_tf import calculate_transformation_matrix

FIXED_ORIENTATION = (-1.57, 0, 1.57)
TRAJECTORY_FILE = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/my_final_project_sim/spiral_trajectory_points.txt"
INITIAL_POSITION = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]  # First input
joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
ROTATION = np.array([[0.001, -0.001, -1.0],
                    [1.0, 0.0, 0.001],
                    [-0.0, -1.0, 0.001]])

def load_trajectory(file_path):
    """ Reads (x, y) trajectory and adds Z_INIT & fixed orientation. """
    X_INIT = -0.1488

    trajectory = []
    with open(file_path, "r") as file:
        for line in file:
            y, z = map(float, line.strip().split(","))
            y /= 1000.0  # Convert mm to meters
            y = y - 0.06333
            z /= 1000.0  # Convert mm to meters
            z = z + 0.21874
            trajectory.append((X_INIT, y, z))
    
    return trajectory

def execute_trajectory():
    """ Executes the trajectory in simulation using inverse kinematics (IK). """
    rospy.init_node('cobot_trajectory_sim', anonymous=True)
    pub = rospy.Publisher('/desired_joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    rospy.sleep(2.0)

    # First publish the initial desired position
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = [
        'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
        'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
    ]
    joint_state.position = INITIAL_POSITION
    pub.publish(joint_state)

    rospy.loginfo(f"Published initial desired joint position: {INITIAL_POSITION}")
    rospy.sleep(5.0)

    # Load and execute trajectory
    full_poses = load_trajectory(TRAJECTORY_FILE)
    if not full_poses:
        rospy.logwarn("No trajectory poses loaded. Exiting...")
        return

    last_joint_angles = INITIAL_POSITION  # Store the last sent pose

    try:
        for pose in full_poses:
            if rospy.is_shutdown():
                rospy.logwarn("ROS shutdown detected. Exiting trajectory execution...")
                break

            if len(pose) == 3:
                x, y, z = pose

                # Construct transformation matrix with predefined rotation and new translation
                target_pose_matrix = np.eye(4)
                target_pose_matrix[:3, :3] = ROTATION
                target_pose_matrix[:3, 3] = [x, y, z]
                rospy.loginfo(target_pose_matrix)
                joint_angles = inverse_kinematics_minimize(target_pose_matrix, last_joint_angles) - joint_offsets

                if joint_angles is not None:
                    last_joint_angles = joint_angles  # Update last known pose
                    joint_state.header.stamp = rospy.Time.now()
                    joint_state.position = last_joint_angles
                    pub.publish(joint_state)
                    rospy.loginfo(f"Published new desired joint positions: {last_joint_angles}")
                    rospy.sleep(10.0)  # Add 5-second delay between two poses
                
            else:
                rospy.logwarn(f"IK failed for pose: {pose}, reusing last known joint angles.")

    except KeyboardInterrupt:
        rospy.logwarn("Trajectory execution interrupted. Exiting...")
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS shutdown requested. Exiting...")

    rospy.loginfo("Trajectory execution completed.")

if __name__ == '__main__':
    try:
        execute_trajectory()
    except KeyboardInterrupt:
        rospy.logwarn("Process interrupted manually. Exiting...")
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS is shutting down. Exiting...")
