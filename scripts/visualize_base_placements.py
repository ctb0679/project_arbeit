#!/usr/bin/env python3
import os

import rospy
import h5py
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


def create_base_marker(spheres, resolution, frame_id="joint1"):
    """
    Create a CUBE_LIST marker for base placement candidates.

    spheres: Nx4 array [x, y, z, quality]
    resolution: voxel size (m)
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "base_placements"
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD

    # Draw thin "tiles" on the ground plane
    marker.scale = Vector3(resolution, resolution, 0.01)
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(0.0)  # forever

    if spheres.size == 0:
        rospy.logwarn("Base placement visualization: empty sphere array.")
        return marker

    # Normalise quality values to [0, 1] for colouring
    q_vals = spheres[:, 3].astype(np.float64)
    q_min = float(q_vals.min())
    q_max = float(q_vals.max())
    denom = (q_max - q_min) if (q_max - q_min) > 1e-9 else 1.0
    norm_q = (q_vals - q_min) / denom  # 0..1

    for idx, sphere in enumerate(spheres):
        x, y, z, _ = sphere
        marker.points.append(Point(float(x), float(y), float(z)))

        q = norm_q[idx]  # 0..1
        # Red (0) -> Yellow (0.5) -> Green (1)
        reach_pct = 100.0 * q
        r = min(max(2.0 * (1.0 - reach_pct / 100.0), 0.0), 1.0)
        g = min(max(2.0 * (reach_pct / 100.0), 0.0), 1.0)

        color = ColorRGBA(r, g, 0.0, 0.8)
        marker.colors.append(color)

    return marker


def load_base_map(h5_path):
    """
    Load base placement 3D map from an HDF5 file created by
    create_base_placement_map_for_goal_pose.py.

    Returns:
        spheres: Nx4 array [x, y, z, quality]
        resolution: voxel size (m)
    """
    if not os.path.isfile(h5_path):
        raise FileNotFoundError("Base map HDF5 file not found: {}".format(h5_path))

    with h5py.File(h5_path, "r") as f:
        if "/Spheres/sphere_dataset" not in f:
            raise KeyError("Could not find /Spheres/sphere_dataset in {}".format(h5_path))
        ds = f["/Spheres/sphere_dataset"]
        spheres = ds[...]
        # Resolution attribute was written when the map was created
        resolution = float(ds.attrs.get("Resolution", 0.02))

    return spheres, resolution


def main():
    rospy.init_node("visualize_base_placements")

    h5_path = rospy.get_param("~map_path", "")
    if not h5_path:
        rospy.logfatal("Parameter '~map_path' is required (path to 3D base map .h5 file).")
        return

    frame_id = rospy.get_param("~frame_id", "joint1")
    max_points = int(rospy.get_param("~max_points", 20000))

    try:
        spheres, resolution = load_base_map(h5_path)
    except Exception as e:
        rospy.logfatal("Failed to load base map from '%s': %s", h5_path, e)
        return

    rospy.loginfo("Loaded base map '%s' with %d spheres (resolution=%.3f m).",
                  h5_path, spheres.shape[0], resolution)

    # Optional downsampling for performance
    if spheres.shape[0] > max_points:
        idx = np.random.choice(spheres.shape[0], max_points, replace=False)
        spheres = spheres[idx]
        rospy.loginfo("Downsampled base map to %d points for visualization.", spheres.shape[0])

    marker_pub = rospy.Publisher("base_placement_map", Marker,
                                 queue_size=1, latch=True)

    rospy.sleep(0.5)  # give publisher some time

    marker = create_base_marker(spheres, resolution, frame_id=frame_id)
    marker_pub.publish(marker)
    rospy.loginfo("Published base placement visualization as topic '%s'.",
                  marker_pub.resolved_name)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
