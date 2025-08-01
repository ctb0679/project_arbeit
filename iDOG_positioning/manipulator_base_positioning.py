import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_commander
import numpy as np
from generate_candidate_position import generate_candidate_positions

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('april_tag_reachability', anonymous=True)

# Setup MoveIt Commander for the arm
arm_group = moveit_commander.MoveGroupCommander("arm")

# Setup tf listener
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

cmd_vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

# Get the AprilTag pose in the camera frame and transform it to the base frame
try:
    transform = tf_buffer.lookup_transform('base_link', 'tag_frame', rospy.Time(0), rospy.Duration(3.0))
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = transform.transform.translation.x
    target_pose.position.y = transform.transform.translation.y
    target_pose.position.z = transform.transform.translation.z
    target_pose.orientation = transform.transform.rotation

    # Set the pose target for the arm
    arm_group.set_pose_target(target_pose)

    # Attempt to plan the motion
    if arm_group.go(wait=True):
        rospy.loginfo("Target reached successfully!")
    else:
        rospy.logwarn("Target unreachable, repositioning the base...")
        # Generate candidate base positions around the target
        radius = 0.5
        num_positions = 8
        candidate_positions = generate_candidate_positions(target_pose, radius, num_positions)

        # Iterate through candidate positions to find a feasible base location
        for candidate_pos in candidate_positions:
            x, y, z = candidate_pos
            # Command the base to move (for simplicity, just moving in straight lines here)
            move_cmd = geometry_msgs.msg.Twist()
            move_cmd.linear.x = x
            move_cmd.linear.y = y
            cmd_vel_pub.publish(move_cmd)
            rospy.sleep(2)  # Move and wait to settle

            # Try to reach the target again
            if arm_group.go(wait=True):
                rospy.loginfo("Target reached after repositioning!")
                break

except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    rospy.logerr(e)

moveit_commander.roscpp_shutdown()
