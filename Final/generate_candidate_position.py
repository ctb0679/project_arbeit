import numpy as np

def generate_candidate_positions(target_pose, radius, num_positions):
    positions = []
    for angle in np.linspace(0, 2 * np.pi, num_positions):
        x = target_pose.position.x + radius * np.cos(angle)
        y = target_pose.position.y + radius * np.sin(angle)
        positions.append((x, y, target_pose.position.z))
    return positions
