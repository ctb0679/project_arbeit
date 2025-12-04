import numpy as np
from matplotlib import pyplot as plt

# Input the center and radius
center = input('Enter the center of the circle (x, y): ').split(',')
center_x, center_y = float(center[0]), float(center[1])
radius = float(input('Enter the radius of the circle: '))

# Input for trajectory spacing
spacing = float(input('Enter the spacing between trajectory lines: '))

# Calculate side lengths
inner_square_side = 2 * radius  # Original square that fits the circle
outer_square_side = inner_square_side + spacing  # New larger square

# Define the outer square coordinates
outer_top_left = (center_x - radius - spacing / 2, center_y + radius + spacing / 2)
outer_bottom_left = (center_x - radius - spacing / 2, center_y - radius - spacing / 2)
outer_top_right = (center_x + radius + spacing / 2, center_y + radius + spacing / 2)
outer_bottom_right = (center_x + radius + spacing / 2, center_y - radius - spacing / 2)

# Generate trajectory points starting from the top-left corner of the outer square
x_trajectory = []
y_trajectory = []
current_y = outer_top_left[1]
moving_right = True  # Flag to track direction

while current_y >= outer_bottom_left[1]:
    if moving_right:
        # Move from left to right
        x_trajectory.extend([outer_top_left[0], outer_top_right[0]])
        y_trajectory.extend([current_y, current_y])
    else:
        # Move from right to left
        x_trajectory.extend([outer_top_right[0], outer_top_left[0]])
        y_trajectory.extend([current_y, current_y])

    # Move downward
    if current_y - spacing >= outer_bottom_left[1]:  # Prevent overshooting
        x_trajectory.append(x_trajectory[-1])  # Maintain x position
        y_trajectory.append(current_y - spacing)

    # Update current_y and direction
    current_y -= spacing
    moving_right = not moving_right  # Flip direction

# Remove excess points to stay within bounds
x_trajectory = np.clip(x_trajectory, outer_top_left[0], outer_top_right[0])
y_trajectory = np.clip(y_trajectory, outer_bottom_left[1], outer_top_left[1])

# Generate points for the circle
theta = np.linspace(0, 2 * np.pi, 100)
x_circle = center_x + radius * np.cos(theta)
y_circle = center_y + radius * np.sin(theta)

# Generate points for the inner square
inner_square_x = [
    center_x - radius, center_x + radius, center_x + radius, center_x - radius, center_x - radius
]
inner_square_y = [
    center_y + radius, center_y + radius, center_y - radius, center_y - radius, center_y + radius
]

# Generate points for the outer square
outer_square_x = [
    outer_top_left[0], outer_top_right[0], outer_bottom_right[0], outer_bottom_left[0], outer_top_left[0]
]
outer_square_y = [
    outer_top_left[1], outer_top_right[1], outer_bottom_right[1], outer_bottom_left[1], outer_top_left[1]
]

# Plot the circle, squares, and trajectory
plt.figure(figsize=(8, 8))
plt.plot(x_circle, y_circle, label='Circle', color='blue')
plt.plot(inner_square_x, inner_square_y, label='Inner Square', color='green')
plt.plot(outer_square_x, outer_square_y, label='Outer Square', color='purple', linestyle='dashed')
plt.plot(x_trajectory, y_trajectory, label='Trajectory', color='orange')
plt.scatter(x_trajectory, y_trajectory, color='red', s=10, label='Trajectory Points')
plt.axis('equal')
plt.title('Circle, Bounding Squares, and Strict Horizontal Zigzag Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
