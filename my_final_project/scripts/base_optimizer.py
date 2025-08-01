#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import nlopt
from numpy import *
from math import cos, sin, atan2
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from forward_kinematics_sim import T_lamb  # Import your FK function
from incremental_ik import incremental_ik  # Your IK solver

class WallAlignedBaseOptimizer:
    def __init__(self):
        rospy.init_node('base_optimizer')
        
        # TF broadcaster for visualization
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Socket pose relative to INITIAL base (MODIFY THESE)
        self.socket_pose_base = Pose(
            position=Point(0.4, 0.4, 0.3),  # X=forward, Y=left, Z=up
            orientation=Quaternion(-np.sqrt(2)/2, 0, 0, np.sqrt(2)/2)  # Identity
        )
        
        # Extract socket's Z-axis
        socket_z = self.get_socket_z_axis()
        
        # Compute angle of socket's Z-axis in XY plane
        socket_z_xy_angle = atan2(socket_z.y, socket_z.x)
        
        # Calculate required yaw so base's +Y aligns with socket's +Z
        self.required_yaw = socket_z_xy_angle - np.pi/2
        
        # Normalize angle
        if self.required_yaw > np.pi:
            self.required_yaw -= 2*np.pi
        elif self.required_yaw < -np.pi:
            self.required_yaw += 2*np.pi
            
        rospy.loginfo(f"Socket Z-axis: ({socket_z.x:.4f}, {socket_z.y:.4f}, {socket_z.z:.4f})")
        rospy.loginfo(f"Required base yaw: {np.degrees(self.required_yaw):.1f}°")
        
        # Optimization parameters
        self.max_xy = 0.5  # Max XY movement (m)
        self.K = 5.0       # Movement penalty weight
        
        # Joint state (start from home)
        self.joint_angles = [0.0] * 6
        
        # Publish TF frames
        self.publish_initial_tf()
        rospy.sleep(1)  # Allow TF propagation
        
        # Run optimization
        self.execute()

    def get_socket_z_axis(self):
        """Extract Z-axis vector from socket orientation"""
        q = self.socket_pose_base.orientation
        rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        return Vector3(*rot_matrix[:, 2])  # Third column is Z-axis

    def publish_initial_tf(self):
        """Publish TF frames for visualization"""
        # Initial base (world origin)
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "base_initial",
            "world"
        )
        
        # Socket relative to initial base
        p = self.socket_pose_base.position
        q = self.socket_pose_base.orientation
        self.tf_broadcaster.sendTransform(
            (p.x, p.y, p.z),
            (q.x, q.y, q.z, q.w),
            rospy.Time.now(),
            "socket",
            "base_initial"
        )

    def transform_socket_to_world(self, base_xy):
        """
        Compute socket pose in world frame after base movement
        base_xy: (x, y) displacement from initial position
        """
        dx, dy = base_xy
        p = self.socket_pose_base.position
        return Pose(
            position=Point(p.x + dx, p.y + dy, p.z),
            orientation=self.socket_pose_base.orientation
        )

    def pose_to_matrix(self, pose):
        """Convert geometry_msgs/Pose to 4x4 transformation matrix"""
        t = pose.position
        q = pose.orientation
        matrix = np.eye(4)
        matrix[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def compute_ik_error(self, base_xy):
        """Compute Cartesian error using FK validation"""
        # 1. Transform socket to world frame
        socket_world = self.transform_socket_to_world(base_xy)
        goal_matrix = self.pose_to_matrix(socket_world)
        
        # 2. Solve IK
        try:
            q_solution = incremental_ik(self.joint_angles, goal_matrix)
        except Exception as e:
            rospy.logwarn(f"IK failed: {str(e)}")
            return float('inf')  # Unreachable
        
        # 3. Compute FK for achieved pose
        achieved_matrix = T_lamb(*q_solution)
        
        # 4. Calculate position error (Euclidean distance)
        goal_pos = goal_matrix[:3, 3]
        achieved_pos = achieved_matrix[:3, 3]
        return np.linalg.norm(goal_pos - achieved_pos)

    def cost_function(self, x, grad):
        """Cost = Movement penalty + IK position error"""
        dx, dy = x
        movement = dx**2 + dy**2
        ik_error = self.compute_ik_error((dx, dy))
        return self.K * movement + ik_error

    def execute(self):
        """Main optimization workflow"""
        # Configure optimizer (X,Y only)
        opt = nlopt.opt(nlopt.LN_COBYLA, 2)
        opt.set_min_objective(self.cost_function)
        opt.set_xtol_rel(0.01)  # 1cm tolerance
        opt.set_lower_bounds([-self.max_xy, -self.max_xy])
        opt.set_upper_bounds([self.max_xy, self.max_xy])
        
        # Run optimization
        try:
            xy_opt = opt.optimize([0.0, 0.0])
            rospy.loginfo(f"Optimal base position: dx={xy_opt[0]:.3f}m, dy={xy_opt[1]:.3f}m")
            
            # Publish optimal base TF (with required yaw)
            self.publish_optimal_base(xy_opt)
            
            # Validate final error
            final_error = self.compute_ik_error(xy_opt)
            rospy.loginfo(f"Final Cartesian error: {final_error:.6f}m")
            
        except Exception as e:
            rospy.logerr(f"Optimization failed: {str(e)}")

    def publish_optimal_base(self, xy_opt):
        """Publish optimal base frame continuously"""
        dx, dy = xy_opt
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransform(
                (dx, dy, 0),
                quaternion_from_euler(0, 0, self.required_yaw),
                rospy.Time.now(),
                "base_optimal",
                "world"
            )
            rate.sleep()

if __name__ == "__main__":
    optimizer = WallAlignedBaseOptimizer()
    rospy.spin()