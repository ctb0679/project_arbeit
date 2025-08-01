import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
import moveit_commander
from scipy.linalg import svd


class BasePositionPlanner:
    def __init__(self, min_distance, max_distance, num_positions):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.num_positions = num_positions
        self.candidate_positions = []
        self.feasible_positions = []
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('april_tag_reachability', anonymous=True)
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    def calculate_wall_normal(self, tag_frame):
        try:
            # Get the transform of the AprilTag frame relative to the world frame
            transform = self.tf_buffer.lookup_transform('world', tag_frame, rospy.Time(0), rospy.Duration(3.0))

            # Extract the rotation as a quaternion
            q = transform.transform.rotation
            quaternion = np.array([q.x, q.y, q.z, q.w])

            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)

            # The Z-axis of the AprilTag frame is assumed to be normal to the wall
            tag_z_axis = np.array([0.0, 0.0, 1.0])  # Z-axis in the tag's local frame
            wall_normal = np.dot(rotation_matrix, tag_z_axis)  # Transform Z-axis to world frame

            # Normalize the wall normal to get a unit vector
            wall_normal = wall_normal / np.linalg.norm(wall_normal)
            return wall_normal
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error calculating wall normal: {e}")
            return np.array([0.0, 0.0, 1.0])  # Default normal vector assuming the wall is perpendicular to the ground

    def quaternion_to_rotation_matrix(self, quaternion):
        # Convert a quaternion to a rotation matrix
        x, y, z, w = quaternion
        rotation_matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
        return rotation_matrix

    def generate_candidate_positions_front_of_wall(self, tag_pose):
        normal = np.array([1.0, 0.0])  # Modify this vector based on your actual wall orientation
        tag_position = np.array([tag_pose.position.x, tag_pose.position.y])

        for distance in np.linspace(self.min_distance, self.max_distance, self.num_positions):
            candidate_position = tag_position + distance * normal
            self.candidate_positions.append((candidate_position[0], candidate_position[1], tag_pose.position.z))

    def check_reachability(self, tag_pose):
        for candidate_pos in self.candidate_positions:
            x, y, z = candidate_pos
            # Command the base to move to the candidate position (not implemented here)

            try:
                # Set the target pose for the arm
                target_pose = geometry_msgs.msg.Pose()
                target_pose.position.x = tag_pose.position.x
                target_pose.position.y = tag_pose.position.y
                target_pose.position.z = tag_pose.position.z
                target_pose.orientation.w = 1.0  # Assuming tag's orientation remains the same

                self.arm_group.set_pose_target(target_pose)
                success = self.arm_group.go(wait=True)

                if success:
                    self.feasible_positions.append(candidate_pos)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def compute_manipulability_index(self, base_position):
        # Placeholder for actual Jacobian computation logic
        J = self.get_jacobian(base_position)  # Assuming this function gives you the Jacobian matrix
        singular_values = svd(J, compute_uv=False)
        manipulability_index = np.prod(singular_values)
        return manipulability_index

    def get_jacobian(self, base_position):
        try:
            # Assume the current joint values of the manipulator are set
            joint_values = self.arm_group.get_current_joint_values()

            # Compute the Jacobian matrix
            jacobian = self.arm_group.get_jacobian_matrix(joint_values)
            return np.array(jacobian)
        except Exception as e:
            rospy.logerr(f"Error computing Jacobian: {e}")
            return np.zeros((6, len(self.arm_group.get_active_joints())))  # Return a zero matrix if an error occurs

    def optimize_base_position(self, tag_pose):
        best_position = None
        best_cost = float('inf')

        for position in self.feasible_positions:
            cost = self.cost_function(position, tag_pose)
            if cost < best_cost:
                best_cost = cost
                best_position = position

        return best_position

    def cost_function(self, base_position, target_position):
        manipulability_cost = -self.compute_manipulability_index(base_position)
        distance_cost = np.linalg.norm(np.array(base_position[:2]) - np.array([target_position.position.x, target_position.position.y]))
        total_cost = manipulability_cost + distance_cost
        return total_cost

    def move_to_position(self, position):
        x, y, _ = position
        move_cmd = geometry_msgs.msg.Twist()
        move_cmd.linear.x = x
        move_cmd.linear.y = y
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(2)  # Wait for the base to settle

    def execute(self, tag_pose):
        self.generate_candidate_positions_front_of_wall(tag_pose)
        self.check_reachability(tag_pose)

        if not self.feasible_positions:
            rospy.logwarn("No feasible positions found.")
            return

        optimal_position = self.optimize_base_position(tag_pose)
        rospy.loginfo(f"Optimal base position found: {optimal_position}")
        self.move_to_position(optimal_position)

        # Re-attempt to reach the target with the arm
        self.arm_group.set_pose_target(tag_pose)
        if self.arm_group.go(wait=True):
            rospy.loginfo("Target reached successfully after repositioning!")
        else:
            rospy.logwarn("Failed to reach the target even after repositioning.")

        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        tag_pose = geometry_msgs.msg.Pose()
        tag_pose.position.x = 1.0
        tag_pose.position.y = 0.0
        tag_pose.position.z = 0.0  # Assuming the tag is at ground level

        planner = BasePositionPlanner(min_distance=0.5, max_distance=1.5, num_positions=5)
        planner.execute(tag_pose)
    except rospy.ROSInterruptException:
        pass


# Once the pose of the April tag has been obtained orient the iDOG not facing the wall (parallel to the wall)
# Now without changing its orientation move the iDOG according to the generated candidate positions
