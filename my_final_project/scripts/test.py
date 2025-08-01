#!/usr/bin/env python3
import rospy
import time
import numpy as np
from mycobot_communication.msg import MycobotAngles
from mycobot_communication.srv import SetAngles

# Constants
TOLERANCE_DEG = 7.0
SPEED = 60  # Safe range: 50–70
WAIT_RATE_HZ = 10

# Globals
current_angles = [None] * 6

def radians_to_degrees(radian_list):
    return [round(r * 180.0 / np.pi, 2) for r in radian_list]

def joint_state_callback(msg):
    global current_angles
    current_angles = [
        msg.joint_1,
        msg.joint_2,
        msg.joint_3,
        msg.joint_4,
        msg.joint_5,
        msg.joint_6,
    ]

def has_reached_target(target_angles, tolerance=TOLERANCE_DEG):
    if None in current_angles:
        return False
    return all(abs(c - t) < tolerance for c, t in zip(current_angles, target_angles))

def send_joint_angles(target_angles_deg, speed=SPEED):
    rospy.wait_for_service('/set_joint_angles')
    try:
        set_angles = rospy.ServiceProxy('/set_joint_angles', SetAngles)
        response = set_angles(*target_angles_deg, speed)
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def load_waypoints(filepath):
    waypoints = []
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != 6:
                continue
            radians = [float(x) for x in parts]
            degrees = radians_to_degrees(radians)
            waypoints.append(degrees)
    return waypoints

def main():
    rospy.init_node("waypoint_service_controller")

    rospy.Subscriber("/mycobot/angles_real", MycobotAngles, joint_state_callback)
    waypoints = load_waypoints('/home/er/inspection_ws/src/scientific-working-ss-2024/my_final_project/generated_joint_states_test.txt')

    if not waypoints:
        rospy.logerr("No valid waypoints found.")
        return

    rate = rospy.Rate(WAIT_RATE_HZ)

    for i, target in enumerate(waypoints):
        rospy.loginfo(f"Sending waypoint {i+1}: {target}")
        send_joint_angles(target, SPEED)

        while not rospy.is_shutdown():
            if has_reached_target(target):
                rospy.loginfo(f"✅ Reached waypoint {i+1}. Holding for 3 seconds...")
                time.sleep(3)
                break
            rospy.loginfo_throttle(1, f"Waiting... current: {current_angles} target: {target}")
            rate.sleep()

    rospy.loginfo("All waypoints completed.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
