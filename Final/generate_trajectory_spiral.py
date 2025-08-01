import numpy as np
import matplotlib.pyplot as plt

# Input the center and radius
center = [0, 0]
center_x, center_y = float(center[0]), float(center[1])
radius = float(input('Enter the radius of the circle: '))

# Input for trajectory spacing
spacing = float(input('Enter the spacing between trajectory loops: '))

# Calculate square dimensions
inner_square_side = 2 * radius  # Original square enclosing the circle
outer_square_side = 2 * (radius + spacing)  # Outer square to start the trajectory

# Define the outer square boundaries
left = center_x - radius - spacing / 2
right = center_x + radius + spacing / 2
top = center_y + radius + spacing / 2
bottom = center_y - radius - spacing / 2

# Generate points for the circle
theta = np.linspace(0, 2 * np.pi, 100)
x_circle = center_x + radius * np.cos(theta)
y_circle = center_y + radius * np.sin(theta)

# Generate points for the inner square
inner_square_x = [center_x - radius, center_x + radius, center_x + radius, center_x - radius, center_x - radius]
inner_square_y = [center_y + radius, center_y + radius, center_y - radius, center_y - radius, center_y + radius]

# Generate points for the outer square
outer_square_x = [center_x - radius - spacing, center_x + radius + spacing, center_x + radius + spacing, center_x - radius - spacing, center_x - radius - spacing]
outer_square_y = [center_y + radius + spacing, center_y + radius + spacing, center_y - radius - spacing, center_y - radius - spacing, center_y + radius + spacing]

# Initialize trajectory lists
x_trajectory = []
y_trajectory = []

# Start from the left side and spiral inward
while left < right and bottom < top:
    # Move right
    x_trajectory.extend(np.linspace(left, right, int((right - left) / spacing) + 1))
    y_trajectory.extend([top] * len(x_trajectory[len(y_trajectory):]))
    
    top -= spacing  # Shrink the boundary downward
    if top < bottom:
        break

    # Move down
    y_trajectory.extend(np.linspace(top, bottom, int((top - bottom) / spacing) + 1))
    x_trajectory.extend([right] * len(y_trajectory[len(x_trajectory):]))

    right -= spacing  # Shrink the boundary inward
    if left > right:
        break

    # Move left
    x_trajectory.extend(np.linspace(right, left, int((right - left) / spacing) + 1))
    y_trajectory.extend([bottom] * len(x_trajectory[len(y_trajectory):]))

    bottom += spacing  # Shrink the boundary upward
    if bottom > top:
        break

    # Move up
    y_trajectory.extend(np.linspace(bottom, top, int((top - bottom) / spacing) + 1))
    x_trajectory.extend([left] * len(y_trajectory[len(x_trajectory):]))

    left += spacing  # Shrink the boundary inward

# Calculate the total trajectory length
trajectory_length = 0
for i in range(1, len(x_trajectory)):
    dx = x_trajectory[i] - x_trajectory[i - 1]
    dy = y_trajectory[i] - y_trajectory[i - 1]
    trajectory_length += np.sqrt(dx**2 + dy**2)

print(f'Total trajectory length: {trajectory_length:.2f} units')

# Save trajectory points to a .txt file
file_name = "/home/junaidali/inspection_ws/src/scientific-working-ss-2024/Final/spiral_trajectory_points_test.txt"
with open(file_name, "w") as f:
    for x, y in zip(x_trajectory, y_trajectory):
        f.write(f"{x:.6f}, {y:.6f}\n")

print(f"Trajectory points saved in {file_name}")

# Plot the circle, squares, and trajectory
plt.figure(figsize=(8, 8))
plt.plot(x_circle, y_circle, label='Circle', color='blue')
plt.plot(inner_square_x, inner_square_y, label='Inner Square', color='green')
plt.plot(outer_square_x, outer_square_y, label='Outer Square', color='purple', linestyle='dashed')
plt.plot(x_trajectory, y_trajectory, label='Spiral Trajectory', color='orange')
plt.scatter(x_trajectory, y_trajectory, color='red', s=10, label='Trajectory Points')
plt.axis('equal')
plt.title('Spiral Trajectory Inside Bounding Square')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
