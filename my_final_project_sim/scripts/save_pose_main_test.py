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

# ---------------------------------------------------------------------------
# Socket & trajectory configuration
# ---------------------------------------------------------------------------

# Trajectory file with scan pattern (dy, dx) in mm in the socket frame
TRAJECTORY_FILE = "/home/junaidali/catkin_ws/src/project_arbeit/my_final_project_sim/spiral_trajectory_points.txt"

# Socket pose in the cobot base frame (UPDATE with your real values)
# Socket frame convention:
#   x_S: left/right
#   y_S: up/down
#   z_S: coming out of the wall
SOCKET_POSITION = (0, 0.20, 0.14)     # [x, y, z] in meters (example)
# SOCKET_ORIENTATION = (0.0, -1.5708, -1.5708)     # [roll, pitch, yaw] in radians (if the socket is in the -y direction from the base)
SOCKET_ORIENTATION = (0.0, -1.5708, +1.5708)  # if the socket is in the +y direction from the base

# Distance along socket z-axis where the tool should hover
# (2 cm in front of the socket along +z_S)
APPROACH_DISTANCE = 0.02  # meters

# Joint limits used by incremental_ik
joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
joint_limits = [[-np.radians(165), np.radians(165)] for _ in range(5)] + [[-np.radians(175), np.radians(175)]]


# ---------------------------------------------------------------------------
# Helper: build EE pose from socket pose
# ---------------------------------------------------------------------------

def compute_ee_pose_for_socket_center(socket_position, socket_orientation,
                                      approach_distance=APPROACH_DISTANCE):
    """
    Compute the desired end-effector pose in the robot base frame such that:
    - The EE is at (0, 0, +approach_distance) in the socket frame.
    - The EE z-axis points *towards* the socket (opposite to socket z-axis).

    Socket frame S:
      x_S: left/right
      y_S: up/down
      z_S: out of wall (towards robot)
    """
    # Base -> Socket
    T_bs = np.array(calculate_transformation_matrix(socket_position, socket_orientation))

    # Socket -> EE: stand 'approach_distance' in front of socket along +z_S
    T_se = np.eye(4)
    T_se[2, 3] = approach_distance

    # 180° rotation about x_S:
    #   x_E =  x_S
    #   y_E = -y_S
    #   z_E = -z_S  (so z_E points into the socket)
    R_flip = np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
    ])
    T_se[:3, :3] = R_flip

    # Base -> EE
    T_be = T_bs @ T_se
    return T_be


# ---------------------------------------------------------------------------
# Trajectory: dy, dx in socket frame
# ---------------------------------------------------------------------------

def load_trajectory(file_path,
                    socket_position=SOCKET_POSITION,
                    socket_orientation=SOCKET_ORIENTATION,
                    approach_distance=APPROACH_DISTANCE,
                    clockwise=False):
    """
    Load a 2D scan / spiral trajectory expressed in the socket frame and
    transform it into the robot base frame.

    File format: each non-empty line "dy,dx" in millimeters (center of socket is 0,0)
      - dy: offset along socket y (up/down)
      - dx: offset along socket x (left/right)
    For every point, the EE stays 'approach_distance' in front of the socket along +z_S.
    """
    # Base -> Socket transform
    T_bs = np.array(calculate_transformation_matrix(socket_position, socket_orientation))

    trajectory = []
    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()
            if not line:
                continue

            dy_mm, dx_mm = map(float, line.split(","))

            # Convert mm -> m
            dy = dy_mm * 1e-3
            dx = dx_mm * 1e-3

            # Point in socket frame: (x_S, y_S, z_S) = (dx, dy, approach_distance)
            p_s = np.array([dx, dy, approach_distance, 1.0])

            # Transform into base frame
            p_b = T_bs @ p_s
            x_b, y_b, z_b = p_b[0], p_b[1], p_b[2]

            trajectory.append((x_b, y_b, z_b))

    # Flip scan direction: anti-clockwise → clockwise
    if clockwise:
        trajectory.reverse()

    return trajectory


# ---------------------------------------------------------------------------
# Collision checking & IK scoring
# ---------------------------------------------------------------------------

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
    score = 0.0
    for i, angle in enumerate(joint_angles):
        lower, upper = joint_limits[i]
        center = (upper + lower) / 2.0
        range_half = (upper - lower) / 2.0
        score += abs(angle - center) / range_half
    return score


def get_best_collision_free_ik(prev_joint_state, pose_matrix, joint_names, group_name):
    """
    Try multiple IK solutions (using incremental_ik) and pick the one that is
    collision-free, within joint limits, and closest to the previous joint state.
    """
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


# ---------------------------------------------------------------------------
# Main execution
# ---------------------------------------------------------------------------

def execute_trajectory():
    rospy.init_node('cobot_trajectory_sim', anonymous=True)
    roscpp_initialize([])
    global collision_service
    RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("mycobot_arm")
    rospy.wait_for_service("/check_state_validity")
    collision_service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

    # ---------------------------------------------------------
    # 1) Compute initial EE pose from socket pose
    # ---------------------------------------------------------
    initial_pose_matrix = compute_ee_pose_for_socket_center(
        SOCKET_POSITION,
        SOCKET_ORIENTATION,
        APPROACH_DISTANCE
    )

    # Use current joint values as seed for IK (or zeros if not available)
    seed_joints = group.get_current_joint_values()
    if not seed_joints:
        seed_joints = [0.0] * 6

    initial_joint_state = get_best_collision_free_ik(
        seed_joints,
        initial_pose_matrix,
        group.get_active_joints(),
        group.get_name()
    )

    if initial_joint_state is None:
        rospy.logerr("No collision-free IK solution found for initial EE pose. Aborting.")
        return

    # Orientation to reuse for all trajectory points
    # (EE z-axis pointing towards socket)
    R_des = initial_pose_matrix[:3, :3]

    # ---------------------------------------------------------
    # 2) Open output file and write initial joint state
    # ---------------------------------------------------------
    output_joint_file = "/home/junaidali/catkin_ws/src/project_arbeit/my_final_project_sim/generated_joint_states_02.12.txt"
    with open(output_joint_file, "w") as f_out:
        rospy.sleep(2.0)

        # Initial configuration from IK
        f_out.write(",".join(str(val) for val in initial_joint_state) + "\n")
        rospy.loginfo(f"Saved initial joint position (from IK): {initial_joint_state}")

        # -----------------------------------------------------
        # 3) Load trajectory (socket frame → base frame)
        # -----------------------------------------------------
        trajectory = load_trajectory(
            TRAJECTORY_FILE,
            SOCKET_POSITION,
            SOCKET_ORIENTATION,
            APPROACH_DISTANCE
        )
        if not trajectory:
            rospy.logwarn("No trajectory points loaded.")
            return

        last_joint_angles = initial_joint_state

        total_points = len(trajectory)
        success_count = 0

        # -----------------------------------------------------
        # 4) Follow trajectory with fixed orientation R_des
        # -----------------------------------------------------
        for x, y, z in trajectory:
            if rospy.is_shutdown():
                break

            # Build pose matrix with fixed orientation (R_des) and varying position
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = R_des
            pose_matrix[:3, 3] = [x, y, z]

            best_ik = get_best_collision_free_ik(
                last_joint_angles,
                pose_matrix,
                group.get_active_joints(),
                group.get_name()
            )

            if best_ik is not None:
                last_joint_angles = best_ik
                f_out.write(",".join(str(val) for val in best_ik) + "\n")
                success_count += 1
                rospy.loginfo(f"Saved new joint state: {best_ik}")
                rospy.sleep(1.5)
            else:
                rospy.logwarn(f"No collision-free IK found for pose: {(x, y, z)}. Using last pose.")
                
            
            # -----------------------------------------------------
            # 5) Report success percentage
            # -----------------------------------------------------
            if total_points > 0:
                success_rate = 100.0 * float(success_count) / float(total_points)
            else:
                success_rate = 0.0

            rospy.loginfo(
                f"IK success rate along trajectory: "
                f"{success_count}/{total_points} poses = {success_rate:.1f}%"

            )


if __name__ == '__main__':
    try:
        execute_trajectory()
    except rospy.ROSInterruptException:
        pass
