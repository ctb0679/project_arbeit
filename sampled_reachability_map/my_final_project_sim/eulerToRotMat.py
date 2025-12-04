from scipy.spatial.transform import Rotation as R
import numpy as np

# XYZ Euler angles
euler_xyz = [-1.57, 0, 1.57]

# Create rotation object in XYZ order
rot = R.from_euler('xyz', euler_xyz, degrees=False)

# Get rotation matrix
R_matrix = rot.as_matrix()
print(np.round(R_matrix, 3))
