#!/usr/bin/env python
# rostopic pub /mycobot/angles_goal mycobot_communication/MycobotSetAngles "{joint_1: -90.0, joint_2: 30.0, joint_3: -53.0, joint_4: 0.0, joint_5: 0.0, joint_6: 0.0, speed: 40}"

import sys

sys.path.append('/home/er/inspection_ws/src/scientific-working-ss-2024/kinematics')

import rospy
import tf2_ros
import numpy as np
from tf.transformations import quaternion_matrix
# from sensor_msgs.msg import JointState
from mycobot_communication.msg import MycobotSetAngles # to publish joint states
from mycobot_communication.msg import MycobotAngles # to subscribe joint states
from std_msgs.msg import Float64MultiArray  
import geometry_msgs.msg  
from kinematics.scripts.ik_node import inverse_kinematics_minimize

current_joint_states = None
joint_offsets = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0]
initial_positions = [0, 0, 0, 0, 0, 0]
# Define joint limits in degrees
joint_limits = {
    "J1": (-165, 165),
    "J2": (-165, 165),
    "J3": (-165, 165),
    "J4": (-165, 165),
    "J5": (-165, 165),
    "J6": (-179, 179)
}
working_radius = 0.280  # in meters (280 mm)

# Function to check joint limits
def check_joint_limits(joint_states_degrees, joint_limits):
    for i, (joint, limits) in enumerate(joint_limits.items()):
        if not (limits[0] <= joint_states_degrees[i] <= limits[1]):
            print(f"{joint} is out of range: {joint_states_degrees[i]}° (Limits: {limits[0]}° to {limits[1]}°)")
            return False
    print("All joint states are within the specified limits.")
    return True

# Reachability check function
def reachability_check(joint_states, end_effector_position, working_radius):
joint_states_degrees = radians_to_degrees(joint_states)

if check_joint_limits(joint_states_degrees, joint_limits):
    distance_from_origin = np.linalg.norm(end_effector_position)
    print("Distance from Origin:", distance_from_origin)

    if distance_from_origin <= working_radius:
        print("The target pose is within the working radius.")
    else:
        print("The target pose is outside the working radius.")
else:
    print("The target pose is not reachable due to joint limits being exceeded.")

# Example joint states in radians
joint_states = [np.radians(30), np.radians(45), np.radians(-160), np.radians(90), np.radians(120), np.radians(-170)]

# Example end-effector position in meters (calculated from FK)
end_effector_position = np.array([0.2, 0.1, 0.25])

# Define the working radius in meters
working_radius = 0.280

# Perform the reachability check
reachability_check(joint_states, end_effector_position, working_radius)

def transform_to_matrix(transform):
    """
    Convert a Transform message to a 4x4 transformation matrix.
    """
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    # Create a 4x4 identity matrix
    transform_matrix = np.identity(4)

    # Set the translation
    transform_matrix[0, 3] = translation.x
    transform_matrix[1, 3] = translation.y
    transform_matrix[2, 3] = translation.z

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

    # Combine translation and rotation into a single transformation matrix
    transform_matrix[:3, :3] = rotation_matrix[:3, :3]

    return transform_matrix

def joint_state_callback(msg):
    global current_joint_states
    current_joint_states = msg
    global initial_positions

    
    initial_positions[0] = np.radians(current_joint_states.joint_1)
    initial_positions[1] = np.radians(current_joint_states.joint_2)
    initial_positions[2] = np.radians(current_joint_states.joint_3)
    initial_positions[3] = np.radians(current_joint_states.joint_4)
    initial_positions[4] = np.radians(current_joint_states.joint_5)
    initial_positions[5] = np.radians(current_joint_states.joint_6)

    for i in range(6):
        initial_positions[i] += joint_offsets[i]

def main():
    rospy.init_node('transform_listener', anonymous=True)

    # Load forward kinematics matrix
    fk_msg = rospy.wait_for_message('fk_matrix', Float64MultiArray)  
    fk_matrix = np.reshape(fk_msg.data, (4, 4))  
    
    # Set up transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Load hand-eye transformation matrix
    matrix_camera_2_ee = np.loadtxt('/home/er/inspection_ws/src/scientific-working-ss-2024/eye_in_hand_calibration/calibration_files/cam2gripper_tf_apriltag_2.txt')
    
    rospy.Subscriber('/mycobot/angles_real', MycobotAngles, joint_state_callback, queue_size=10) # callback function?

    # Set up publisher for desired joint states
    desired_joint_state_pub = rospy.Publisher('/mycobot/angles_goal', MycobotSetAngles, queue_size=10)
    
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            # Get the transform from the camera to the AprilTag
            transform_camera_to_apriltag = tfBuffer.lookup_transform('usb_cam', 'tag_5', rospy.Time(0))
            
            translation = transform_camera_to_apriltag.transform.translation
            rotation = transform_camera_to_apriltag.transform.rotation

            # rospy.loginfo(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
            # rospy.loginfo(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")
            
            end_effector_position = np.array([translation.x, translation.y, translation.z])

            distance_from_origin = np.linalg.norm(end_effector_position)
            print("distance: ", distance_from_origin)
            

            if distance_from_origin <= working_radius:
                print("The target pose is within the working radius.")
            else:
                print("The target pose is outside the working radius.")

            
            
            # Convert transforms to matrices
            matrix_ee_2_base = fk_matrix
            matrix_apriltag_2_camera = transform_to_matrix(transform_camera_to_apriltag)
            
            # Compute the result matrix: base_to_ee * ee_to_camera * camera_to_apriltag
            matrix_camera_2_base = np.dot(matrix_ee_2_base, matrix_camera_2_ee)
            result_matrix_apriltag_2_base = np.dot(matrix_camera_2_base, matrix_apriltag_2_camera) # apriltag in base frame

            # rospy.loginfo("Transformation Matrix (end_effector to base):\n%s", matrix_ee_2_base)
            # rospy.loginfo("Transformation Matrix (camera to end_effector):\n%s", matrix_camera_2_ee)
            # rospy.loginfo("Transformation Matrix (apriltag to camera):\n%s", matrix_apriltag_2_camera)
            # rospy.loginfo("Transformation Matrix (camera to base):\n%s", matrix_camera_2_base)
            # rospy.loginfo("Transformation Matrix (apriltag to base):\n%s", result_matrix_apriltag_2_base)
            
            matrix_apriltag_camera_final_pose = result_matrix_apriltag_2_base
            matrix_apriltag_camera_final_pose[1,3] += 0.2
            rospy.loginfo("Transformation Matrix final pose:\n%s", matrix_apriltag_camera_final_pose)
            

            
            

            
            if current_joint_states is not None:
                # Placeholder for inverse kinematics function
                print("current js:",  initial_positions)
                print("target:",  matrix_apriltag_camera_final_pose)
                desired_joint_angles = inverse_kinematics_minimize(result_matrix_apriltag_2_base, initial_positions)- joint_offsets
                print("Desired angles: ", desired_joint_angles)
                rospy.loginfo(desired_joint_angles)

                desired_joint_angles.joint_1 = np.rad2deg(desired_joint_angles[0])
                desired_joint_angles.joint_2 = np.rad2deg(desired_joint_angles[1])
                desired_joint_angles.joint_3 = np.rad2deg(desired_joint_angles[2])
                desired_joint_angles.joint_4 = np.rad2deg(desired_joint_angles[3])
                desired_joint_angles.joint_5 = np.rad2deg(desired_joint_angles[4])
                desired_joint_angles.joint_6 = np.rad2deg(desired_joint_angles[5])

                if desired_joint_angles.joint_1 
                '''
                # Create JointState message and publish it
                desired_joint_state_msg = MycobotSetAngles()
                desired_joint_state_msg.joint_1 = np.rad2deg(desired_joint_angles[0])
                desired_joint_state_msg.joint_2 = np.rad2deg(desired_joint_angles[1])
                desired_joint_state_msg.joint_3 = np.rad2deg(desired_joint_angles[2])
                desired_joint_state_msg.joint_4 = np.rad2deg(desired_joint_angles[3])
                desired_joint_state_msg.joint_5 = np.rad2deg(desired_joint_angles[4])
                desired_joint_state_msg.joint_6 = np.rad2deg(desired_joint_angles[5])
                desired_joint_state_msg.speed = 50
  

                desired_joint_state_pub.publish(desired_joint_state_msg)
                rospy.loginfo("Published desired joint states: %s", desired_joint_angles)
                '''
                # break
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform not available: %s", e)


if __name__ == '__main__':
    main()

