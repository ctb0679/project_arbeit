import os
import math
import time
import pickle
import h5py
import numpy as np
import torch
from datetime import datetime
from symb_mat_read import T_lamb  # Your FK implementation

# ----------------------------
# CONFIGURATION (CPU OPTIMIZED)
# ----------------------------
device = "cpu"  # Force CPU usage
dtype = torch.float32
print(f"Using device: {device}")

# Joint limits in radians (mycobot280pi)
joint_limits_deg = [
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-175, 175]
]

joint_pos_min = torch.tensor([math.radians(x[0]) for x in joint_limits_deg], 
                          dtype=dtype, device=device)
joint_pos_max = torch.tensor([math.radians(x[1]) for x in joint_limits_deg], 
                          dtype=dtype, device=device)
n_dof = len(joint_pos_min)

# Conservative workspace boundaries (will expand after first run)
x_lim = [-0.25, 0.25]  # Meters
y_lim = [-0.25, 0.25]
z_lim = [0.0, 0.3]     # Base to max reach height

# Orientation ranges
r_lim = [-math.pi, math.pi]    # Roll
p_lim = [-math.pi, math.pi]    # Pitch
yaw_lim = [-math.pi, math.pi]  # Yaw

# Map resolution (coarser for first run)
cartesian_res = 0.02  # 2 cm resolution
angular_res = math.pi/6  # 30 degrees per bin

# ----------------------------
# REACHABILITY MAP SETTINGS
# ----------------------------
# Calculate bin counts
x_bins = math.ceil((x_lim[1] - x_lim[0]) / cartesian_res)
y_bins = math.ceil((y_lim[1] - y_lim[0]) / cartesian_res)
z_bins = math.ceil((z_lim[1] - z_lim[0]) / cartesian_res)
roll_bins = math.ceil((r_lim[1] - r_lim[0]) / angular_res)
pitch_bins = math.ceil((p_lim[1] - p_lim[0]) / angular_res)
yaw_bins = math.ceil((yaw_lim[1] - yaw_lim[0]) / angular_res)

# Reduced sampling for CPU
N_fk = 100000  # Total samples (100K - adjust based on performance)
num_loops = 10  # Process in batches
N_fk_loop = N_fk // num_loops
save_freq = 1  # Save every loop since batches are small

# File paths
reach_map_file_path = os.getcwd() + '/reachability_maps/'
os.makedirs(reach_map_file_path, exist_ok=True)
timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
reach_map_file_name = f'mycobot_reach_map_{cartesian_res}_{timestamp}'

# ----------------------------
# CORE FUNCTIONS (CPU OPTIMIZED)
# ----------------------------
def batch_forward_kinematics(th_batch):
    """Compute FK for a batch of joint angles"""
    batch_size = th_batch.shape[0]
    matrices = torch.zeros((batch_size, 4, 4), dtype=dtype)
    
    # Convert to numpy for your FK function
    th_np = th_batch.numpy()
    
    for i in range(batch_size):
        T_np = T_lamb(*th_np[i])
        matrices[i] = torch.tensor(T_np, dtype=dtype)
    
    return matrices

def numerical_jacobian(th_batch):
    """Numerical Jacobian computation for CPU"""
    batch_size = th_batch.shape[0]
    J = torch.zeros((batch_size, 6, n_dof), dtype=dtype)
    eps = 1e-6
    
    base_poses = batch_forward_kinematics(th_batch)
    base_pos = base_poses[:, :3, 3]
    base_rot = base_poses[:, :3, :3]
    
    for j in range(n_dof):
        # Perturb joint j
        th_pert = th_batch.clone()
        th_pert[:, j] += eps
        
        # Compute perturbed poses
        pert_poses = batch_forward_kinematics(th_pert)
        pert_pos = pert_poses[:, :3, 3]
        pert_rot = pert_poses[:, :3, :3]
        
        # Position Jacobian
        J[:, :3, j] = (pert_pos - base_pos) / eps
        
        # Orientation Jacobian (simplified)
        rot_diff = pert_rot - base_rot
        J[:, 3:, j] = rot_diff.view(batch_size, 9)[:, [2, 5, 8]]  # Approximate
    
    return J

def compute_manipulability(J):
    """Compute position-based manipulability measure"""
    J_pos = J[:, :3, :]
    if J_pos.shape[-1] < 3:
        return torch.zeros(J_pos.shape[0], dtype=dtype)
    
    # Compute JJᵀ and its determinant
    JJT = torch.bmm(J_pos, J_pos.transpose(1, 2))
    dets = torch.det(JJT)
    
    # Handle negative determinants (shouldn't occur but for safety)
    dets = torch.abs(dets)
    return torch.sqrt(dets)

# ----------------------------
# REACHABILITY MAP INIT
# ----------------------------
# Create 6D reachability map
num_voxels = x_bins * y_bins * z_bins * roll_bins * pitch_bins * yaw_bins
print(f"Total voxels: {num_voxels:,}")
print(f"Estimated memory: {(num_voxels * 8 * 8) / (1024**2):.2f} MB")  # 8 values * 8 bytes

# Only store non-zero entries to save memory
reach_dict = {}  # {linear_index: [position, orientation, count, max_manip]}

# Indexing offsets
offsets = torch.tensor([
    y_bins * z_bins * roll_bins * pitch_bins * yaw_bins,
    z_bins * roll_bins * pitch_bins * yaw_bins,
    roll_bins * pitch_bins * yaw_bins,
    pitch_bins * yaw_bins,
    yaw_bins,
    1
], dtype=torch.long)

# ----------------------------
# SAMPLING AND PROCESSING
# ----------------------------
print(f"Starting reachability mapping for {N_fk} samples...")
print(f"Workspace: X{x_lim}, Y{y_lim}, Z{z_lim}")
print(f"Orientation: R{r_lim}, P{p_lim}, Y{yaw_lim}")
start_time = time.time()

for i in range(num_loops):
    loop_start = time.time()
    
    # Sample joint configurations
    th_batch = (joint_pos_max - joint_pos_min) * torch.rand(N_fk_loop, n_dof, dtype=dtype) + joint_pos_min
    
    # Compute forward kinematics
    ee_transf_batch = batch_forward_kinematics(th_batch)
    
    # Extract position and orientation
    positions = ee_transf_batch[:, :3, 3]
    orientations = torch.stack([
        torch.atan2(ee_transf_batch[:, 2, 1], ee_transf_batch[:, 2, 2]),
        torch.asin(-ee_transf_batch[:, 2, 0]),
        torch.atan2(ee_transf_batch[:, 1, 0], ee_transf_batch[:, 0, 0])
    ], dim=1)
    
    poses_6d = torch.cat((positions, orientations), dim=1)
    
    # Discretize poses
    lower_limits = torch.tensor(
        [x_lim[0], y_lim[0], z_lim[0], r_lim[0], p_lim[0], yaw_lim[0]],
        dtype=dtype
    )
    resolutions = torch.tensor(
        [cartesian_res, cartesian_res, cartesian_res, 
         angular_res, angular_res, angular_res],
        dtype=dtype
    )
    
    indices_6d = (poses_6d - lower_limits) / resolutions
    indices_6d = torch.floor(indices_6d).long()
    
    # Clip indices to valid range
    dim_limits = torch.tensor([x_bins, y_bins, z_bins, roll_bins, pitch_bins, yaw_bins])
    indices_6d = torch.clamp(indices_6d, 0, dim_limits - 1)
    
    # Compute linear indices
    linear_indices = torch.sum(indices_6d * offsets, dim=1).numpy()
    
    # Compute manipulability (using numerical Jacobian)
    if i == 0:  # Only compute for first batch to save time
        J = numerical_jacobian(th_batch)
        M = compute_manipulability(J).numpy()
    else:
        M = np.zeros(N_fk_loop)  # Dummy values after first batch
    
    # Update reachability dictionary
    for idx, pose, m_val in zip(linear_indices, poses_6d.numpy(), M):
        if idx in reach_dict:
            reach_dict[idx][2] += 1  # Increase count
            if m_val > reach_dict[idx][3]:
                reach_dict[idx][3] = m_val  # Update max manipulability
            # Update average pose (optional)
            # reach_dict[idx][0] = (reach_dict[idx][0]*reach_dict[idx][2] + pose[:3])/(reach_dict[idx][2]+1)
        else:
            # [position, orientation, count, max_manip]
            reach_dict[idx] = [pose[:3], pose[3:], 1, m_val]
    
    # Save intermediate results
    loop_time = time.time() - loop_start
    total_time = time.time() - start_time
    print(f"Iter {i+1}/{num_loops} | "
          f"Loop: {loop_time:.1f}s | "
          f"Total: {total_time:.1f}s | "
          f"Points: {len(reach_dict)}")
    
    if i % save_freq == 0:
        # Convert to array: [x, y, z, roll, pitch, yaw, count, max_manip]
        data = []
        for idx, (pos, ori, count, manip) in reach_dict.items():
            data.append(np.concatenate([pos, ori, [count], [manip]]))
        
        reach_array = np.array(data)
        with open(f"{reach_map_file_path}{reach_map_file_name}_iter{i}.pkl", 'wb') as f:
            pickle.dump(reach_array, f)

# ----------------------------
# FINAL SAVE AND VISUALIZATION
# ----------------------------
# Convert to final array
final_data = []
for idx, (pos, ori, count, manip) in reach_dict.items():
    final_data.append(np.concatenate([pos, ori, [count], [manip]]))

reach_array = np.array(final_data)

# Save final map
with open(f"{reach_map_file_path}{reach_map_file_name}_final.pkl", 'wb') as f:
    pickle.dump(reach_array, f)

# Create 3D visualization map
print("Creating 3D visualization map...")
if len(reach_array) > 0:
    # Group by position voxels
    positions = reach_array[:, :3]
    manip_scores = reach_array[:, -1]
    
    # Find unique positions and max manipulability per position
    unique_pos, inverse = np.unique(positions, axis=0, return_inverse=True)
    max_manip = np.zeros(len(unique_pos))
    for i in range(len(unique_pos)):
        mask = (inverse == i)
        max_manip[i] = np.max(manip_scores[mask]) if np.any(mask) else 0
    
    # Create sphere data: [x, y, z, score]
    sphere_data = np.column_stack((unique_pos, max_manip))
    
    # Save as HDF5
    with h5py.File(f"{reach_map_file_path}3D_{reach_map_file_name}.h5", 'w') as f:
        sphere_group = f.create_group('/Spheres')
        sphere_dset = sphere_group.create_dataset('sphere_dataset', data=sphere_data)
        sphere_dset.attrs['Resolution'] = cartesian_res
        
        # Save sample poses
        pose_group = f.create_group('/Poses')
        sample_poses = []
        for pos in unique_pos:
            # Find first pose at this position
            idx = np.where((positions == pos).all(axis=1))[0][0]
            sample_poses.append(reach_array[idx, :6])
        pose_dset = pose_group.create_dataset('poses_dataset', data=np.array(sample_poses))
else:
    print("Warning: No reachability data generated!")

total_time = time.time() - start_time
print(f"Completed in {total_time:.1f} seconds")
print(f"Results saved to: {reach_map_file_path}")
print(f"Reachable points: {len(reach_dict)}")