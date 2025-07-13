import numpy as np

# Parameters
map_width_meters = 50  # width of the map in meters
map_height_meters = 100  # height of the map in meters
resolution = 0.1  # resolution in meters (10 cm per cell)
outer_ellipse_width = 40  # width of the outer ellipse in meters
outer_ellipse_height = 90  # height of the outer ellipse in meters
wall_thickness = 0.5  # thickness of the outer wall in meters
track_thickness = 6  # thickness of the track in meters
inner_wall_thickness = 0.5  # thickness of the inner wall in meters (facing inwards)

# Calculate grid size
map_width_cells = int(map_width_meters / resolution)
map_height_cells = int(map_height_meters / resolution)

# Ellipse parameters for the outer ellipse (semi-major and semi-minor axes)
a_outer = outer_ellipse_width / 2  # Outer ellipse (horizontal axis)
b_outer = outer_ellipse_height / 2  # Outer ellipse (vertical axis)

# Ellipse parameters for the inner ellipse (semi-major and semi-minor axes)
a_inner = a_outer - track_thickness  # Inner ellipse (horizontal axis)
b_inner = b_outer - track_thickness  # Inner ellipse (vertical axis)

# Initialize map (all cells as unknown, i.e., -1)
grid = np.ones((map_height_cells, map_width_cells), dtype=int) * -1

# First pass: Fill the track with 0 (inside the ellipses)
for i in range(map_height_cells):
    for j in range(map_width_cells):
        # Convert pixel (i, j) to meters
        x = (j * resolution) - (map_width_meters / 2)  # x-coordinate in meters
        y = (i * resolution) - (map_height_meters / 2)  # y-coordinate in meters
        
        # Fill track area (0) between the two ellipses
        if (x / a_outer) ** 2 + (y / b_outer) ** 2 <= 1 and (x / a_inner) ** 2 + (y / b_inner) ** 2 > 1:
            grid[i, j] = 0  # Track area

# Second pass: Add the walls
for i in range(map_height_cells):
    for j in range(map_width_cells):
        # Convert pixel (i, j) to meters
        x = (j * resolution) - (map_width_meters / 2)  # x-coordinate in meters
        y = (i * resolution) - (map_height_meters / 2)  # y-coordinate in meters
        
        # Outer wall (100) around the outer ellipse (0.5 meters thick)
        if (x / a_outer) ** 2 + (y / b_outer) ** 2 <= 1 and (x / a_outer) ** 2 + (y / b_outer) ** 2 > ((a_outer - wall_thickness) / a_outer) ** 2:
            grid[i, j] = 100  # Outer wall

        # Inner wall (100) inside the inner ellipse (0.5 meters thick, facing inward)
        if (x / a_inner) ** 2 + (y / b_inner) ** 2 <= 1 and (x / a_inner) ** 2 + (y / b_inner) ** 2 > ((a_inner - inner_wall_thickness) / a_inner) ** 2:
            grid[i, j] = 100  # Inner wall

# Save the grid to a text file
file_path = "oval_track.txt"
np.savetxt(file_path, grid, fmt='%d')

print(f"Map saved as {file_path}")