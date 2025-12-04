#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from incremental_ik import incremental_ik, generate_random_vector, check_joint_vector_within_limits
# from test_incremental_ik import incremental_ik, check_joint_vector_within_limits
from pose_and_matrix_tf import calculate_transformation_matrix
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState as RobotStateMsg

FIXED_ORIENTATION = (-1.57, 0, 1.57)
TRAJECTORY_FILE = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/my_final_project_sim/spiral_trajectory_points.txt"
INITIAL_POSITION = [0.0, 0.523599, 2.0944, -2.61799, 3.14, 0.0]
# INITIAL_POSITION = [0.0, 1.0472, 1.5708, -2.61799, 3.14, 0.0]
joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
joint_limits = [[-np.radians(165), np.radians(165)] for _ in range(5)] + [[-np.radians(175), np.radians(175)]]


def load_trajectory(file_path):
    # X_INIT = -0.18921
    X_INIT = -0.1488
    trajectory = []
    with open(file_path, "r") as file:
        for line in file:
            y, z = map(float, line.strip().split(","))
            y /= 1000.0
            y -= 0.06333
            z /= 1000.0
            # z += 0.17833
            z += 0.21874
            trajectory.append((X_INIT, y, z))
    return trajectory


def is_collision_free(joint_names, joint_positions, group_name):
    rs = RobotStateMsg()
    rs.joint_state.name = joint_names
    rs.joint_state.position = joint_positions
    req = GetStateValidityRequest()
    req.robot_state = rs
    req.group_name = group_name
    result = collision_service(req)
    return result.valid


def joint_distance_score(joint_a, joint_b):
    return np.linalg.norm(np.array(joint_a) - np.array(joint_b))


def proximity_to_limits_score(joint_angles):
    score = 0
    for i, angle in enumerate(joint_angles):
        lower, upper = joint_limits[i]
        center = (upper + lower) / 2
        range_half = (upper - lower) / 2
        score += abs(angle - center) / range_half
    return score


def get_best_collision_free_ik(prev_joint_state, pose_matrix, joint_names, group_name):
    candidates = []
    for _ in range(10):
        ik_sol = incremental_ik(prev_joint_state, pose_matrix)
        if not check_joint_vector_within_limits(ik_sol, joint_limits):
            continue
        if is_collision_free(joint_names, ik_sol, group_name):
            dist_score = joint_distance_score(prev_joint_state, ik_sol)
            limit_score = proximity_to_limits_score(ik_sol)
            total_score = dist_score + limit_score
            candidates.append((total_score, ik_sol))

    if candidates:
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]
    else:
        return None



def execute_trajectory():
    rospy.init_node('cobot_trajectory_sim', anonymous=True)
    roscpp_initialize([])
    global collision_service
    RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("mycobot_arm")
    rospy.wait_for_service("/check_state_validity")
    collision_service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

    output_joint_file = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/my_final_project_sim/generated_joint_states_test.txt"
    with open(output_joint_file, "w") as f_out:
        rospy.sleep(2.0)
        f_out.write(",".join([str(val) for val in INITIAL_POSITION]) + "\n")
        rospy.loginfo(f"Saved initial joint position: {INITIAL_POSITION}")

        trajectory = load_trajectory(TRAJECTORY_FILE)
        if not trajectory:
            rospy.logwarn("No trajectory points loaded.")
            return

        last_joint_angles = INITIAL_POSITION

        for pose in trajectory:
            if rospy.is_shutdown():
                break
            x, y, z = pose
            pose_matrix = calculate_transformation_matrix((x, y, z), FIXED_ORIENTATION)
            best_ik = get_best_collision_free_ik(last_joint_angles, np.array(pose_matrix), group.get_active_joints(), group.get_name())
            if best_ik:
                last_joint_angles = best_ik
                f_out.write(",".join([str(val) for val in best_ik]) + "\n")
                rospy.loginfo(f"Saved new joint state: {best_ik}")
                rospy.sleep(1.5)
            else:
                rospy.logwarn(f"No collision-free IK found for pose: {pose}. Using last pose.")
                f_out.write(",".join([str(val) for val in last_joint_angles]) + "\n")

if __name__ == '__main__':
    try:
        execute_trajectory()
    except rospy.ROSInterruptException:
        pass
