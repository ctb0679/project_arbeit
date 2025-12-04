import os
from datetime import datetime
import pickle
import h5py

import math
import numpy as np
import pytorch_kinematics as pk
import torch

import time
import pdb

### Code to create a reachability map using pytorch forward kinematics (GPU-based tensor calculations)

# Use CUDA if available
if torch.cuda.is_available():
    d = "cuda"
    torch.cuda.empty_cache()
    print("[GPU MEMORY size in GiB]: " + str((torch.cuda.get_device_properties(0).total_memory - torch.cuda.memory_reserved(0)) / 1024**3))
else:
    d = "cpu"
dtype = torch.float32  # Choose float32 or 64 etc.

## Settings for the reachability map:
robot_urdf = "/home/idac/Junaidali/catkin_ws/src/sampled_reachability_map/scripts/mycobot_expanded.urdf"     # <-- expanded URDF (no xacro tags)
name_end_effector = "joint6_flange"
name_base_link = "joint1"
use_torso = False
n_dof = 6  # myCobot 280 Pi

# Number of DOFs and joint limits (degrees -> radians)
joint_pos_min = torch.deg2rad(torch.tensor([
    -165.0, -165.0, -165.0, -165.0, -165.0, -175.0
], dtype=dtype, device=d))
joint_pos_max = torch.deg2rad(torch.tensor([
     165.0,  165.0,  165.0,  165.0,  165.0,  175.0
], dtype=dtype, device=d))

joint_pos_centers = joint_pos_min + (joint_pos_max - joint_pos_min) / 2
joint_pos_range_sq = (joint_pos_max - joint_pos_min).pow(2) / 4

## Build kinematic chain from URDF
print("[Building kinematic chain from URDF...]:\n...\n...")
chain = pk.build_serial_chain_from_urdf(open(robot_urdf).read(), name_end_effector)
chain = chain.to(dtype=dtype, device=d)
assert (len(chain.get_joint_parameter_names()) == n_dof), "Incorrect number of DOFs set"
print("...\n...")

# Number of Forward Kinematic solutions to sample
N_fk = 10000000  # practical start for 6-DOF small arm

# Map resolution and limits
angular_res = np.pi / 8  # 22.5 degrees per bin
r_lim = [-np.pi, np.pi]  # Using 'intrinsic' euler rotations in XYZ
p_lim = [-np.pi / 2, np.pi / 2]
yaw_lim = [-np.pi, np.pi]
roll_bins = math.ceil((2 * np.pi) / angular_res)   # 16
pitch_bins = math.ceil((np.pi) / angular_res)      # 8 (half elevation)
yaw_bins = math.ceil((2 * np.pi) / angular_res)    # 16

# Compact workspace for 280 mm reach (tweak as needed)
x_lim = [-0.25, 0.25]
y_lim = [-0.25, 0.25]
z_lim = [ 0.00, 0.35]
cartesian_res = 0.02  # 2 cm

x_bins = math.ceil((x_lim[1] - x_lim[0]) / cartesian_res)
y_bins = math.ceil((y_lim[1] - y_lim[0]) / cartesian_res)
z_bins = math.ceil((z_lim[1] - z_lim[0]) / cartesian_res)
post_process = True  # Whether to filter out voxels with post processing

## Create 6D reachability map initialized with zeros
# NOTE: Save map in CPU. Save GPU memory for kinematics, jacobian calculations
num_voxels = x_bins * y_bins * z_bins * roll_bins * pitch_bins * yaw_bins
num_values = 6 + 2  # 6D pose + [Visitation, Manipulability]
reach_map = torch.zeros((num_voxels, num_values), dtype=dtype, device="cpu")
print("[Number of 6D Voxels possible (Based on resolution settings)]: " + str(num_voxels))

# Full path and file name to save
reach_map_file_path ='/home/idac/Junaidali/catkin_ws/src/sampled_reachability_map/maps/'
os.makedirs(reach_map_file_path, exist_ok=True)  # <-- NEW: ensure folder exists
reach_map_file_name = 'reach_map_' + str(name_end_effector) + '_samples_' + str(N_fk) + '_' + str(cartesian_res) + '_' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

# Offsets for indexing the map
x_ind_offset = y_bins * z_bins * roll_bins * pitch_bins * yaw_bins
y_ind_offset = z_bins * roll_bins * pitch_bins * yaw_bins
z_ind_offset = roll_bins * pitch_bins * yaw_bins
roll_ind_offset = pitch_bins * yaw_bins
pitch_ind_offset = yaw_bins
yaw_ind_offset = 1
offsets = torch.tensor([x_ind_offset, y_ind_offset, z_ind_offset, roll_ind_offset, pitch_ind_offset, yaw_ind_offset], dtype=torch.long, device=d)

## Set number of sampled joint configurations as per GPU memory limitations
num_loops = 100
N_fk_loop = int(N_fk / num_loops)
save_freq = int(num_loops / 100)  # save every loop in this config
sampling_distr = torch.distributions.uniform.Uniform(joint_pos_min, joint_pos_max)

## Compute
print("[Starting FK and Jacobian calculations...]")
print("[Number of loops is: " + str(num_loops) + "]")
print("[Map save frequency is " + str(save_freq) + " loops]")
print(f"[Will save file named: '{reach_map_file_name}' at path: '{reach_map_file_path}']")
t0 = time.perf_counter()

for i in range(num_loops):
    loop_t0 = time.perf_counter()

    # Sample joints
    th_batch = sampling_distr.sample([N_fk_loop])

    # FK
    ee_transf_batch = chain.forward_kinematics(th_batch).get_matrix()
    torch.cuda.empty_cache()

    # 6D poses = [x y z r p y]
    poses_6d = torch.hstack((
        ee_transf_batch[:, :3, 3],
        pk.transforms.matrix_to_euler_angles(ee_transf_batch[:, :3, :3], 'XYZ')
    ))

    # Discretization (indices before clamping/masking)
    indices_6d = poses_6d - torch.tensor(
        [x_lim[0], y_lim[0], z_lim[0], r_lim[0], p_lim[0], yaw_lim[0]],
        dtype=dtype, device=d
    )
    indices_6d /= torch.tensor(
        [cartesian_res, cartesian_res, cartesian_res, angular_res, angular_res, angular_res],
        dtype=dtype, device=d
    )
    indices_6d = torch.floor(indices_6d)

    # Clamp angle bins to upper edges
    indices_6d[indices_6d[:, 3] >= roll_bins, 3] = roll_bins - 1
    indices_6d[indices_6d[:, 4] >= pitch_bins, 4] = pitch_bins - 1
    indices_6d[indices_6d[:, 5] >= yaw_bins, 5] = yaw_bins - 1

    # --- NEW: discard out-of-range samples (cartesian + angular bins)
    valid_mask = (
        (indices_6d[:, 0] >= 0) & (indices_6d[:, 0] < x_bins) &
        (indices_6d[:, 1] >= 0) & (indices_6d[:, 1] < y_bins) &
        (indices_6d[:, 2] >= 0) & (indices_6d[:, 2] < z_bins) &
        (indices_6d[:, 3] >= 0) & (indices_6d[:, 3] < roll_bins) &
        (indices_6d[:, 4] >= 0) & (indices_6d[:, 4] < pitch_bins) &
        (indices_6d[:, 5] >= 0) & (indices_6d[:, 5] < yaw_bins)
    )
    if valid_mask.sum() == 0:
        del ee_transf_batch
        torch.cuda.empty_cache()
        continue

    # Keep only valid rows for indices, poses, AND joints (so Jacobian aligns)
    indices_6d = indices_6d[valid_mask]
    poses_6d   = poses_6d[valid_mask]
    th_batch   = th_batch[valid_mask]  # <-- NEW

    # (Optional) warnings for debug visibility
    if (torch.sum((indices_6d[:, 3] >= roll_bins) | (indices_6d[:, 4] >= pitch_bins) | (indices_6d[:, 5] >= yaw_bins) |
                  (indices_6d[:, 0] >= x_bins) | (indices_6d[:, 1] >= y_bins) | (indices_6d[:, 2] >= z_bins)) > 0):
        print("[WARNING] Overflow at higher end after masking (should be rare).")
    if (torch.sum(indices_6d < 0) > 0):
        print("[WARNING] Overflow at lower end after masking (should be rare).")

    # Discretize poses to voxel centers (works on masked indices)
    poses_6d = indices_6d * torch.tensor(
        [cartesian_res, cartesian_res, cartesian_res, angular_res, angular_res, angular_res],
        dtype=dtype, device=d
    )
    poses_6d += torch.tensor(
        [(cartesian_res / 2) + x_lim[0], (cartesian_res / 2) + y_lim[0], (cartesian_res / 2) + z_lim[0],
         (angular_res / 2) + r_lim[0], (angular_res / 2) + p_lim[0], (angular_res / 2) + yaw_lim[0]],
        dtype=dtype, device=d
    )

    # Convert to flat indices (masked)
    indices_6d = indices_6d.to(dtype=torch.long)
    indices_6d = (indices_6d[:, 5] * offsets[5] + indices_6d[:, 4] * offsets[4] +
                  indices_6d[:, 3] * offsets[3] + indices_6d[:, 2] * offsets[2] +
                  indices_6d[:, 1] * offsets[1] + indices_6d[:, 0] * offsets[0])
    indices_6d = indices_6d.cpu()
    poses_6d = poses_6d.cpu()

    del ee_transf_batch
    torch.cuda.empty_cache()

    # Jacobian and manipulability (masked joint batch)
    J = chain.jacobian(th_batch)
    torch.cuda.empty_cache()
    M = torch.det(J @ torch.transpose(J, 1, 2)).cpu()
    del J
    torch.cuda.empty_cache()

    # Add computed pose and manipulability to Reachability Map
    reach_map[indices_6d, :6] = poses_6d  # Save pose at appropriate index

    # Visitation and max manipulability (use torch.maximum to avoid numpy<->torch mixing)
    reach_map[indices_6d, -2] += 1
    reach_map[indices_6d, -1] = torch.maximum(reach_map[indices_6d, -1], M)

    if i % save_freq == 0:
        # Print loop computation time
        t_comp = time.perf_counter() - loop_t0
        print("Loop: " + str(i) + ". Comp Time = {0:.9e}s".format(t_comp))

        # Save reachability map to file (as numpy pkl)
        nonzero_rows = torch.abs(reach_map).sum(dim=1) > 0
        reach_map_nonzero = reach_map[nonzero_rows].numpy()

        with open(reach_map_file_path + reach_map_file_name + '.pkl', 'wb') as f:
            pickle.dump(reach_map_nonzero, f)  # Save only non-zero entries

        # Accumulate 6D voxel scores into 3D sphere scores (for visualization)
        indx = 0
        first = True
        while (indx < reach_map_nonzero.shape[0]):
            sphere_3d = reach_map_nonzero[indx][:3]
            # Count num_repetitions of current 3D sphere (in the next z_ind_offset subarray)
            num_repetitions = (reach_map_nonzero[indx:indx + z_ind_offset][:, :3] == sphere_3d).all(axis=1).sum().astype(dtype=np.int16)
            # Store sphere and average manipulability as the score. (Also, scale by a factor)
            Manip_scaling = 500
            Manip_avg = reach_map_nonzero[indx:indx + num_repetitions, 7].mean() * Manip_scaling
            if first:
                first = False
                sphere_array = np.append(reach_map_nonzero[indx][:3], Manip_avg)
                pose_array = np.append(reach_map_nonzero[0, :6], np.array([0., 0., 0., 1.])).astype(np.single)  # dummy value
            else:
                sphere_array = np.vstack((sphere_array, np.append(reach_map_nonzero[indx][:3], Manip_avg)))
                pose_array = np.vstack((pose_array, np.append(reach_map_nonzero[indx, :6], np.array([0., 0., 0., 1.])).astype(np.single)))  # dummy value
            indx += num_repetitions

        # Save 3D map as hdf5 (Mimic reuleux data structure)
        with h5py.File(reach_map_file_path + "3D_" + reach_map_file_name + ".h5", 'w') as f:
            sphereGroup = f.create_group('/Spheres')
            sphereDat = sphereGroup.create_dataset('sphere_dataset', data=sphere_array)
            sphereDat.attrs.create('Resolution', data=cartesian_res)
            # (Optional) Save all the 6D poses in each 3D sphere. Currently only dummy pose values (10 dimensional)
            poseGroup = f.create_group('/Poses')
            poseDat = poseGroup.create_dataset('poses_dataset', dtype=float, data=pose_array)

## Post processing: (Optional)
if post_process:
    # Remove points that are on or below ground (simple optimistic filter)
    reach_map_filtered = reach_map_nonzero[reach_map_nonzero[:, 2] > 0]

    # Save filtered reach_map and 3D viz map
    with open(reach_map_file_path + 'filt_' + reach_map_file_name + '.pkl', 'wb') as f:
        pickle.dump(reach_map_filtered, f)

    # Accumulate 6D voxel scores into 3D sphere scores (for visualization)
    indx = 0
    first = True
    while (indx < reach_map_filtered.shape[0]):
        sphere_3d = reach_map_filtered[indx][:3]
        num_repetitions = (reach_map_filtered[indx:indx + z_ind_offset][:, :3] == sphere_3d).all(axis=1).sum().astype(dtype=np.int16)
        Manip_scaling = 500
        Manip_avg = reach_map_filtered[indx:indx + num_repetitions, 7].mean() * Manip_scaling
        if first:
            first = False
            sphere_array = np.append(reach_map_filtered[indx][:3], Manip_avg)
            pose_array = np.append(reach_map_filtered[0, :6], np.array([0., 0., 0., 1.])).astype(np.single)  # dummy value
        else:
            sphere_array = np.vstack((sphere_array, np.append(reach_map_filtered[indx][:3], Manip_avg)))
            pose_array = np.vstack((pose_array, np.append(reach_map_filtered[indx, :6], np.array([0., 0., 0., 1.])).astype(np.single)))  # dummy value
        indx += num_repetitions

    with h5py.File(reach_map_file_path + "filt_3D_" + reach_map_file_name + ".h5", 'w') as f:
        sphereGroup = f.create_group('/Spheres')
        sphereDat = sphereGroup.create_dataset('sphere_dataset', data=sphere_array)
        sphereDat.attrs.create('Resolution', data=cartesian_res)
        poseGroup = f.create_group('/Poses')
        poseDat = poseGroup.create_dataset('poses_dataset', dtype=float, data=pose_array)

# END
t_comp = time.perf_counter() - t0
print("[TOTAL Comp Time] = {0:.2e}s".format(t_comp))

# Debug:
# pdb.set_trace()
