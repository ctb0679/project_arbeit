#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

NUM_STEPS = 100

def publish_joint_states():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(60)

    joint_state = JointState()
    joint_state.header = Header()

    # Get initial joint angles from current robot state
    initial_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    initial_positions = list(initial_msg.position)
    rospy.loginfo(f"Initial positions: {initial_positions}")

    # Load waypoints from text file (comma-separated values)
    waypoints = []
    waypoint_file_path = '/home/junaidali/inspection_ws/src/scientific-working-ss-2024/my_final_project_sim/generated_joint_states_test.txt'  # <-- Replace this with actual path

    with open(waypoint_file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 6:
                waypoint = [float(x) for x in parts]
                waypoints.append(waypoint)

    if not waypoints:
        rospy.logerr("No waypoints loaded from the file!")
        return
    
    sequence = 0
    try:
        for idx, desired_positions in enumerate(waypoints):
            rospy.loginfo(f"Moving to waypoint {idx+1}: {desired_positions}")

            for step in range(NUM_STEPS + 1):
                interpolated_positions = [
                    initial_positions[i] + (desired_positions[i] - initial_positions[i]) * step / NUM_STEPS
                    for i in range(len(initial_positions))
                ]

                joint_state.header.seq = sequence
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = [
                    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
                ]
                joint_state.position = interpolated_positions
                joint_state.velocity = []
                joint_state.effort = []

                pub.publish(joint_state)
                sequence += 1
                rate.sleep()

            # Hold for 3 seconds while actively publishing the final pose
            rospy.loginfo(f"Holding at waypoint {idx+1} for 3 seconds...")
            hold_start_time = rospy.Time.now()
            while (rospy.Time.now() - hold_start_time).to_sec() < 3.0 and not rospy.is_shutdown():
                joint_state.header.seq = sequence
                joint_state.header.stamp = rospy.Time.now()
                joint_state.position = desired_positions
                pub.publish(joint_state)
                sequence += 1
                rate.sleep()

            initial_positions = desired_positions
    except KeyboardInterrupt:
        rospy.logwarn("Trajectory execution interrupted. Exiting...")
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS shutdown requested. Exiting...")
        
    rospy.loginfo("Final waypoint reached. Holding position...")
    try: 
        while not rospy.is_shutdown():
            joint_state.header.seq = sequence
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position = initial_positions
            pub.publish(joint_state)
            sequence += 1
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Final hold interrupted. Exiting cleanly.")
        
if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher', anonymous=True)
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
