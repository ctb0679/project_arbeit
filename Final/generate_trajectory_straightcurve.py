import numpy as np
import matplotlib.pyplot as plt

# Function to generate a semicircle transition
def semicircle_transition(start_x, start_y, direction, spacing, num_points=5):
    """
    Generates a semicircle with given spacing at the end of a horizontal movement.
    
    :param start_x: The x-coordinate where the horizontal line ends
    :param start_y: The y-coordinate where the transition starts
    :param direction: +1 for right-side transition, -1 for left-side transition
    :param spacing: The diameter of the semicircle
    :param num_points: Number of points for smoothness
    :return: x, y coordinates of the semicircle
    """
    radius = spacing / 2
    x_center = start_x # Center is shifted outward
    y_center = start_y - radius  # Center is halfway down

    theta = np.linspace(-np.pi/2, np.pi/2, num_points)  # Semicircle curve
    x = x_center + radius * np.cos(theta) * direction  # Flip for left-side transition
    y = y_center + radius * np.sin(theta)

    return x, y

# Input the center and radius
center = input('Enter the center of the circle (x, y): ').split(',')
center_x, center_y = float(center[0]), float(center[1])
radius = float(input('Enter the radius of the circle: '))

# Input for trajectory spacing
spacing = float(input('Enter the spacing between trajectory lines: '))

# Calculate side lengths
inner_square_side = 2 * radius  # Inner square (fits circle)
outer_square_side = inner_square_side + spacing * 2  # Larger square

# Define the outer square boundaries
left = center_x - radius - spacing / 2
right = center_x + radius + spacing / 2
top = center_y + radius + spacing / 2
bottom = center_y - radius - spacing / 2

# Define the outer square coordinates
outer_top_left = (center_x - radius - spacing / 2, center_y + radius + spacing / 2)
outer_bottom_left = (center_x - radius - spacing / 2, center_y - radius - spacing / 2)
outer_top_right = (center_x + radius + spacing / 2, center_y + radius + spacing / 2)
outer_bottom_right = (center_x + radius + spacing / 2, center_y - radius - spacing / 2)

# Generate trajectory points starting from the top-left corner of the outer square
x_trajectory = []
y_trajectory = []
current_y = outer_top_left[1]
current_y_new = outer_top_left[1]
moving_right = False  # Flag to track direction
x_start_initial = outer_top_left[0]
x_stop_final = outer_bottom_right[0]

x_stop = outer_top_right[0] - spacing / 2  # Stop before the right edge
x_trajectory.extend(np.linspace(x_start_initial, x_stop, int ((x_stop - x_start_initial) / spacing) + 1))

y_trajectory.extend([current_y, current_y])
current_y -= spacing
current_y_new = current_y

while current_y_new >= outer_bottom_left[1]:

    current_y = current_y_new
    current_y_new = current_y_new - spacing

    if moving_right:
        # Move from left to right (stop before right side)
        x_start = outer_top_left[0] + spacing / 2
        x_stop = outer_top_right[0] - spacing / 2  # Stop before the right edge
        x_trajectory.extend([x_start, x_stop])
        y_trajectory.extend([current_y, current_y])

        if current_y_new > outer_bottom_left[1]:
            # Semicircle transition at the right edge
            curve_x, curve_y = semicircle_transition(x_stop, current_y, direction=+1, spacing=spacing)
            # Append semicircle transition
            x_trajectory.extend(curve_x)
            y_trajectory.extend(curve_y)

        

    else:
        # Move from right to left (stop before left side)
        x_start = outer_top_right[0] - spacing / 2
        x_stop = outer_top_left[0] + spacing / 2  # Stop before the left edge
        x_trajectory.extend(np.linspace(x_start, x_stop, int ((x_start - x_stop) / spacing) + 1))
        y_trajectory.extend([current_y] * len(x_trajectory[len(y_trajectory):]))

        if current_y_new > outer_bottom_left[1]:
            # Semicircle transition at the right edge
            curve_x, curve_y = semicircle_transition(x_stop, current_y, direction=+1, spacing=spacing)
            # Append semicircle transition
            x_trajectory.extend(curve_x)
            y_trajectory.extend(curve_y)

    moving_right = not moving_right  # Flip direction

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

# Plot the circle, squares, and trajectory
plt.figure(figsize=(8, 8))
plt.plot(x_circle, y_circle, label='Circle', color='blue')
plt.plot(inner_square_x, inner_square_y, label='Inner Square', color='green')
plt.plot(outer_square_x, outer_square_y, label='Outer Square', color='purple', linestyle='dashed')
plt.plot(x_trajectory, y_trajectory, label='Trajectory', color='orange')
plt.scatter(x_trajectory, y_trajectory, color='red', s=10, label='Trajectory Points')
plt.axis('equal')
plt.title('Circle, Bounding Squares, and Semicircular Trajectory Transitions')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
