import os
import argparse
from datetime import datetime
import pickle
import h5py

import math
import numpy as np
from pytorch_kinematics.transforms.rotation_conversions import matrix_to_quaternion, quaternion_to_matrix, euler_angles_to_matrix, matrix_to_euler_angles
import torch

import time
import pdb

### Code to create a base inverse reachability map (using pytorch kinematics)
### Creates a base map conditioned on the goal pose for the robot (i.e. where to place the base given a goal pose for the arm of the robot)
### NOTE: Requires an inverse reachability map and goal pose as input

# NOTE: For the base map, here we switch to INTRINSIC ZYX (or EXTRINSIC XYZ) Euler angles for easier selection of base points on the ground with no roll and no pitch
goal_pose = torch.tensor([0.0, 1.0, 0.28, 0.0, -np.pi/2, -np.pi/2]) # euler angles in INTRINSIC ZYX (radians) TODO: Add quaternion support
ang_thresh = np.pi/8 # threshold for limiting roll and pitch roll angles on the base ground map
dtype = torch.float32 # Choose float32 or 64 etc.
use_vis_freq = True # Use Visitation frequency as M_score for 3D map (and not Manipulability)

## 6D reachability map settings
angular_res = np.pi/8 # or 22.5 degrees per bin)
yaw_lim = [-np.pi, np.pi]
p_lim = [-np.pi/2, np.pi/2]
r_lim = [-np.pi, np.pi]
yaw_bins = math.ceil((2*np.pi)/angular_res)  # 16
pitch_bins = math.ceil((np.pi)/angular_res)  # 8. Only half the bins needed (half elevation and full azimuth sufficient to cover sphere)
roll_bins = math.ceil((2*np.pi)/angular_res) # 16
cartesian_res = 0.02 # metres
Q_scaling = 100

# Full path and file name to save
parser = argparse.ArgumentParser("create base map")
parser.add_argument("--inv_map_pkl", type=str, required=True, help="Filename (with path) of the inv reachability map")
args, unknown = parser.parse_known_args()
inv_reach_map_file_path = os.path.dirname(args.inv_map_pkl)+'/'
inv_reach_map_file_name = os.path.basename(args.inv_map_pkl)
base_map_file_name = 'base_' + inv_reach_map_file_name


t0 = time.perf_counter()
## Load map
with open(args.inv_map_pkl,'rb') as f:
    loaded_dict = pickle.load(f)
    inv_transf_batch = loaded_dict['inv_transf_batch']
    M_scores = loaded_dict['M_scores']
    if ("Vis_freq" in loaded_dict.keys()) and use_vis_freq:
        M_scores = loaded_dict['Vis_freq']
        Manip_scaling = 6
    else:
        Manip_scaling = 500

## Transform map by goal_pose
goal_transf = torch.zeros((4,4))
goal_transf[:3,:3] = euler_angles_to_matrix(goal_pose[3:6], 'ZYX') # Intrinsic ZYX
goal_transf[:,-1] = torch.hstack((goal_pose[:3],torch.tensor(1.0)))
goal_transf = goal_transf.repeat(inv_transf_batch.shape[0], 1, 1)
base_transf_batch = torch.bmm(goal_transf,inv_transf_batch) # NOTE: bradcasting by yourself and using bmm is much faster than matmul or @!
## Slice the base map to only include poses on the ground (z=0)
ground_ind = (base_transf_batch[:,2,3] > (-cartesian_res/2)) & (base_transf_batch[:,2,3] <= (cartesian_res/2))
base_transf_batch = base_transf_batch[ground_ind]
M_scores = M_scores[ground_ind]
## Slice the base map to only include poses with small roll and pitch (within ang_thresh)
base_poses_6d = torch.hstack((base_transf_batch[:,:3,3],matrix_to_euler_angles(base_transf_batch[:,:3,:3],'ZYX'))) # Intrinsic ZYX
filtered_ind = (base_poses_6d[:,4:6] > (-ang_thresh)).all(axis=1) & (base_poses_6d[:,4:6] <= (ang_thresh)).all(axis=1)
base_poses_6d = base_poses_6d[filtered_ind].numpy()
base_transf_batch = base_transf_batch[filtered_ind].numpy()
M_scores = M_scores[filtered_ind].numpy()

## Save base map and Manipulability scores to file (as numpy pkl)
with open(inv_reach_map_file_path+base_map_file_name,'wb') as f:
    save_dict = {"base_poses_6d": base_poses_6d, "M_scores":M_scores}
    pickle.dump(save_dict,f)
print(f"[Saved file: {base_map_file_name} ]")

## Create a 3D viz map (with numpy)
# Discretize poses
indices_6d = base_poses_6d / np.array([cartesian_res,cartesian_res,cartesian_res,angular_res,angular_res,angular_res], dtype=np.single)
indices_6d = np.floor(indices_6d) # Floor to get the appropriate discrete indices
base_poses_6d = indices_6d*np.array([cartesian_res,cartesian_res,cartesian_res,angular_res,angular_res,angular_res], dtype=np.single)
base_poses_6d += np.array([cartesian_res/2,cartesian_res/2,cartesian_res/2,angular_res/2,angular_res/2,angular_res/2], dtype=np.single)
# Loop over all spheres and get 3D sphere array
first = True
for indx in range(base_poses_6d.shape[0]):
    curr_sphere_3d = base_poses_6d[indx,:3]
    if first:
        first = False
        sphere_indxs = (curr_sphere_3d == base_poses_6d[:,:3]).all(axis=1)
        Manip = M_scores[sphere_indxs].mean()
        # Manip = M_scores[sphere_indxs].max()*Manip_scaling # Optional: take max instead of mean
    
        sphere_array = np.expand_dims(np.append(curr_sphere_3d, Manip),0)
        pose_array = np.append(base_poses_6d[0,:6], np.array([0., 0., 0., 1.])).astype(np.single) # dummy value
    else:
        # Check if curr_sphere already exists in the array. If so, skip.
        if((curr_sphere_3d == sphere_array[:,:3]).all(axis=1).any()):
            continue
        
        sphere_indxs = (curr_sphere_3d == base_poses_6d[:,:3]).all(axis=1)
        Manip = M_scores[sphere_indxs].mean()
        # Manip = M_scores[sphere_indxs].max()*Manip_scaling # Optional: take max instead of mean
    
        sphere_array = np.vstack((sphere_array, np.append(curr_sphere_3d, Manip)))
        pose_array = np.vstack((pose_array, np.append(base_poses_6d[indx,:6], np.array([0., 0., 0., 1.])).astype(np.single))) # dummy value

# ===================== NEW (wall filter): derive wall from goal EE pose =====================
# Assumptions:
# - Your goal pose is the EE pose that must reach the AprilTag.
# - The AprilTag center lies 3 cm *behind* the EE along the goal's +Z axis (normal to the wall).
# - The wall plane is the tag's XY plane with normal = goal +Z.
try:
    if 'sphere_array' in locals() and sphere_array is not None and len(sphere_array) > 0:
        # Get goal origin/rotation again (from goal_pose)
        goal_xyz = goal_pose[:3].numpy().astype(float)
        R_goal   = euler_angles_to_matrix(goal_pose[3:6], 'ZYX').numpy()
        z_axis   = R_goal[:,2]
        z_axis   = z_axis / (np.linalg.norm(z_axis) + 1e-12)

        WALL_NORMAL = -z_axis                        # outward (into free space)
        WALL_ORIGIN = goal_xyz + 0.03 * z_axis      # tag center on the wall (3 cm behind EE)

        CLEARANCE = 0.13  # meters: base radius + margin
        ground_z  = 0.0   # your slice already keeps z≈0; explicit here for clarity

        # Build base points in 3D (x,y at ground)
        base_pts_3d = np.column_stack([
            sphere_array[:,0], sphere_array[:,1], np.full((len(sphere_array),), ground_z, dtype=float)
        ])

        # (A) Half-space: keep bases with signed distance >= CLEARANCE (in front of the wall + margin)
        signed_dist = (base_pts_3d - WALL_ORIGIN) @ WALL_NORMAL
        keep_mask = signed_dist >= CLEARANCE

        # (B) Segment-plane test: drop bases where straight line base->goal crosses the wall between them
        dir_vec = (goal_xyz[None,:] - base_pts_3d)           # [N,3]
        denom   = dir_vec @ WALL_NORMAL                      # n·(goal - b)
        num     = - ((base_pts_3d - WALL_ORIGIN) @ WALL_NORMAL)  # -n·(b - p0)
        with np.errstate(divide='ignore', invalid='ignore'):
            t_star = num / denom
        crosses = (denom != 0.0) & (t_star > 0.0) & (t_star < 1.0)

        # Final filter:
        keep_mask = keep_mask & (~crosses)

        kept_before = len(sphere_array)
        sphere_array = sphere_array[keep_mask]
        kept_after  = len(sphere_array)
        print(f"[INFO] Wall filter kept {kept_after}/{kept_before} candidates (CLEARANCE={CLEARANCE:.2f} m).")

        if kept_after == 0:
            print("[INFO] All candidate base poses were filtered by wall constraints for this goal.")
except Exception as _e:
    print(f"[WARN] Wall filter skipped due to error: {_e}")
# =================== END NEW (wall filter) =====================

# === NEW: Print top-K best base placements by quality ===
try:
    TOPK = 10  # change if you want more/less
    # Guard: ensure sphere_array exists and is non-empty
    if 'sphere_array' not in locals() or sphere_array is None or len(sphere_array) == 0:
        print("[INFO] No base placements to rank/print (sphere_array empty).")
    else:
        # Sort by the last column (assumed to be quality) in descending order
        order = np.argsort(sphere_array[:, -1])[::-1]
        top_idx = order[:min(TOPK, len(order))]
        print(f"\n[Top {len(top_idx)} base placements by quality]")
        for i, idx in enumerate(top_idx, start=1):
            row = sphere_array[idx]
            # Best-effort pretty print: assume columns ~ [x, y, yaw, (maybe z?), ..., score]
            score = row[-1]
            # If the array looks like [x,y,yaw,score]
            if sphere_array.shape[1] >= 4:
                x = row[0]; y = row[1]
                # If a yaw column exists, try to interpret it as the third column
                yaw = row[2] if sphere_array.shape[1] >= 4 else float('nan')
                print(f"{i:2d}. x={x:+.3f}, y={y:+.3f}, yaw={yaw:+.3f}, score={score:.6f}")
            else:
                # Fallback: print whole row if layout is different
                print(f"{i:2d}. row={row}")
except Exception as e:
    print(f"[WARN] Could not rank/print base placements: {e}")

# # Optional: Normalize Q values in the map
min_Q = sphere_array[:,-1].min()
max_Q = sphere_array[:,-1].max()
sphere_array[:,-1] -= min_Q
sphere_array[:,-1] /= (max_Q-min_Q)
sphere_array[:,-1] *= Q_scaling

# Save 3D map as hdf5 (Mimic reuleux data structure)
with h5py.File(inv_reach_map_file_path+"3D_"+base_map_file_name+".h5", 'w') as f:
    sphereGroup = f.create_group('/Spheres')
    sphereDat = sphereGroup.create_dataset('sphere_dataset', data=sphere_array)
    sphereDat.attrs.create('Resolution', data=cartesian_res)
    # (Optional) Save all the 6D poses in each 3D sphere. Currently only dummy pose values (10 dimensional)
    poseGroup = f.create_group('/Poses')
    poseDat = poseGroup.create_dataset('poses_dataset', dtype=float, data=pose_array)
print(f"[Saved file: 3D_{base_map_file_name}.h5 ]")


# END
t_comp = time.perf_counter() - t0
print("[TOTAL Comp Time] = {0:.2e}s".format(t_comp))

# Debug: Time perf counter
# pdb.set_trace()
# tmat = time.perf_counter()

# t_comp = time.perf_counter() - tmat
# print("Comp Time = {0:.9e}s".format(t_comp))
# pdb.set_trace()
