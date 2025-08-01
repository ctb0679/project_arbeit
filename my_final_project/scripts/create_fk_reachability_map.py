import os
import math
import time
import pickle
import h5py
import numpy as np
import torch
from datetime import datetime
from symb_mat_read import T_lamb  # Your FK implementation

# Use CUDA if available
device = "cuda" if torch.cuda.is_available() else "cpu"
dtype = torch.float32

# ----------------------------
# ROBOT SPECIFIC CONFIGURATION
# ----------------------------
# Joint limits in radians
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
n_dof = 6

# Workspace boundaries
x_lim = [-0.3, 0.3]  # Meters (adjust after initial tests)
y_lim = [-0.3, 0.3]
z_lim = [0.0, 0.4]

# Orientation ranges
r_lim = [-math.pi, math.pi]    # Roll
p_lim = [-math.pi, math.pi]    # Pitch
yaw_lim = [-math.pi, math.pi]  # Yaw

# Map resolution
cartesian_res = 0.01  # 1 cm resolution
angular_res = math.pi/12  # 15 degrees per bin

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

# Sampling parameters
N_fk = 100000  # Total samples (10 million)
num_loops = 10   # Process in batches
N_fk_loop = N_fk // num_loops
save_freq = max(1, num_loops // 10)

# File paths
reach_map_file_path = os.getcwd() + '/reachability_maps/'
os.makedirs(reach_map_file_path, exist_ok=True)
timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
reach_map_file_name = f'mycobot_reach_map_{cartesian_res}_{timestamp}'

# ----------------------------
# CORE FUNCTIONS
# ----------------------------
def batch_forward_kinematics(th_batch):
    """Compute FK for a batch of joint angles using your custom function"""
    batch_size = th_batch.shape[0]
    matrices = torch.zeros((batch_size, 4, 4), device=device, dtype=dtype)
    
    # Convert to CPU numpy for your FK function
    th_np = th_batch.cpu().numpy()
    
    for i in range(batch_size):
        T_np = T_lamb(*th_np[i])
        matrices[i] = torch.tensor(T_np, device=device, dtype=dtype)
    
    return matrices

def compute_manipulability(jacobian):
    """Compute Yoshikawa manipulability measure"""
    J = jacobian[:, :3, :]  # Position Jacobian only
    if J.shape[-1] < 3:  # Handle under-actuated cases
        return torch.zeros(J.shape[0], device=device)
    return torch.sqrt(torch.det(J @ J.transpose(1, 2)))

# ----------------------------
# REACHABILITY MAP INIT
# ----------------------------
# Create 6D reachability map
num_voxels = x_bins * y_bins * z_bins * roll_bins * pitch_bins * yaw_bins
num_values = 6 + 2  # Pose + frequency + manipulability
reach_map = torch.zeros((num_voxels, num_values), dtype=dtype, device="cpu")

# Indexing offsets
offsets = torch.tensor([
    y_bins * z_bins * roll_bins * pitch_bins * yaw_bins,
    z_bins * roll_bins * pitch_bins * yaw_bins,
    roll_bins * pitch_bins * yaw_bins,
    pitch_bins * yaw_bins,
    yaw_bins,
    1
], dtype=torch.long, device=device)

# ----------------------------
# SAMPLING AND PROCESSING
# ----------------------------
print(f"Starting reachability mapping for {N_fk} samples...")
print(f"Workspace: X{x_lim}, Y{y_lim}, Z{z_lim}")
print(f"Orientation: R{r_lim}, P{p_lim}, Y{yaw_lim}")
print(f"Map dimensions: {x_bins}x{y_bins}x{z_bins} "
      f"(cartesian) + {roll_bins}x{pitch_bins}x{yaw_bins} (angular)")

sampling_distr = torch.distributions.uniform.Uniform(joint_pos_min, joint_pos_max)
start_time = time.time()

for i in range(num_loops):
    loop_start = time.time()
    
    # Sample joint configurations
    th_batch = sampling_distr.sample([N_fk_loop])
    
    # Compute forward kinematics
    ee_transf_batch = batch_forward_kinematics(th_batch)
    
    # Extract position and orientation (RPY)
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
        device=device, dtype=dtype
    )
    resolutions = torch.tensor(
        [cartesian_res, cartesian_res, cartesian_res, 
         angular_res, angular_res, angular_res],
        device=device, dtype=dtype
    )
    
    indices_6d = (poses_6d - lower_limits) / resolutions
    indices_6d = torch.floor(indices_6d).long()
    
    # Clip indices to valid range
    dim_limits = torch.tensor(
        [x_bins, y_bins, z_bins, roll_bins, pitch_bins, yaw_bins],
        device=device
    )
    indices_6d = torch.clamp(indices_6d, 0, dim_limits - 1)
    
    # Compute linear indices
    linear_indices = torch.sum(indices_6d * offsets, dim=1).cpu()
    
    # Compute manipulability (using PyTorch kinematics-style approximation)
    # This is simplified - consider numerical Jacobian for better accuracy
    J_approx = torch.autograd.functional.jacobian(
        lambda x: batch_forward_kinematics(x.unsqueeze(0))[0, :3, 3],
        th_batch
    ).squeeze(1)
    M = compute_manipulability(J_approx).cpu()
    
    # Update reachability map
    unique_indices, counts = torch.unique(linear_indices, return_counts=True)
    
    # Update visitation frequency
    reach_map[unique_indices, 6] += counts.float()
    
    # Update max manipulability
    for idx, m_val in zip(unique_indices, M):
        if m_val > reach_map[idx, 7]:
            reach_map[idx, 7] = m_val
    
    # Save intermediate results
    if i % save_freq == 0:
        elapsed = time.time() - loop_start
        total_elapsed = time.time() - start_time
        print(f"Iter {i}/{num_loops} | "
              f"Loop: {elapsed:.2f}s | "
              f"Total: {total_elapsed:.2f}s")
        
        # Save checkpoint
        nonzero_mask = reach_map[:, 6] > 0
        reach_map_nonzero = reach_map[nonzero_mask].numpy()
        with open(f"{reach_map_file_path}{reach_map_file_name}_iter{i}.pkl", 'wb') as f:
            pickle.dump(reach_map_nonzero, f)

# ----------------------------
# FINAL SAVE AND POST-PROCESSING
# ----------------------------
# Save final map
nonzero_mask = reach_map[:, 6] > 0
reach_map_nonzero = reach_map[nonzero_mask].numpy()
with open(f"{reach_map_file_path}{reach_map_file_name}_final.pkl", 'wb') as f:
    pickle.dump(reach_map_nonzero, f)

# Create 3D visualization map
print("Creating 3D visualization map...")
positions = reach_map_nonzero[:, :3]
scores = reach_map_nonzero[:, 7]  # Manipulability scores

# Save as HDF5
with h5py.File(f"{reach_map_file_path}3D_{reach_map_file_name}.h5", 'w') as f:
    sphere_group = f.create_group('/Spheres')
    sphere_data = np.column_stack((positions, scores))
    sphere_dset = sphere_group.create_dataset('sphere_dataset', data=sphere_data)
    sphere_dset.attrs['Resolution'] = cartesian_res
    
    # Optional: Save pose information
    pose_group = f.create_group('/Poses')
    # For simplicity, save first pose per position
    _, unique_idx = np.unique(positions, axis=0, return_index=True)
    sample_poses = reach_map_nonzero[unique_idx, :6]
    pose_dset = pose_group.create_dataset('poses_dataset', data=sample_poses)

print(f"Completed in {time.time()-start_time:.2f} seconds")
print(f"Results saved to: {reach_map_file_path}")