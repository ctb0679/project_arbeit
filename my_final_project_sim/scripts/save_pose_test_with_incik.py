#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ik_node import inverse_kinematics_minimize  # Your IK solver
from pose_and_matrix_tf import calculate_transformation_matrix
from incremental_ik import incremental_ik

FIXED_ORIENTATION = (-1.57, 0, 1.57)
TRAJECTORY_FILE = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/Final/spiral_trajectory_points.txt"
INITIAL_POSITION = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]  # First input
joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]

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
    """ Executes the trajectory and saves joint angles to a file instead of publishing. """
    rospy.init_node('cobot_trajectory_sim', anonymous=True)

    output_joint_file = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/Final/generated_joint_states_test_new.txt"
    with open(output_joint_file, "w") as f_out:
        rospy.sleep(2.0)

        # First save the initial desired position
        f_out.write(",".join([str(val) for val in INITIAL_POSITION]) + "\n")
        rospy.loginfo(f"Saved initial desired joint position: {INITIAL_POSITION}")
        rospy.sleep(2.0)

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
                    target_pose_matrix = calculate_transformation_matrix(pose, FIXED_ORIENTATION)
                    rospy.loginfo(target_pose_matrix)
                    joint_angles = incremental_ik(last_joint_angles, np.array(target_pose_matrix))

                    if joint_angles is not None:
                        last_joint_angles = joint_angles  # Update last known pose
                        f_out.write(",".join([str(val) for val in last_joint_angles]) + "\n")
                        rospy.loginfo(f"Saved new desired joint positions: {last_joint_angles}")
                        rospy.sleep(2.0)

                else:
                    rospy.logwarn(f"IK failed for pose: {pose}, reusing last known joint angles.")

        except KeyboardInterrupt:
            rospy.logwarn("Trajectory execution interrupted. Exiting...")
        except rospy.ROSInterruptException:
            rospy.logwarn("ROS shutdown requested. Exiting...")

    rospy.loginfo(f"Trajectory joint angles saved to: {output_joint_file}")

if __name__ == '__main__':
    try:
        execute_trajectory()
    except KeyboardInterrupt:
        rospy.logwarn("Process interrupted manually. Exiting...")
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS is shutting down. Exiting...")
