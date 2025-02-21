# -*- coding: utf-8 -*-
import cv2
import numpy as np
import json
import math
import heapq
import threading
import os
from queue import Queue



# Convert game format (e.g., "A1") to pixel coordinates
def game_to_coords(game_pos, width, height):
    col_part = game_pos[0].upper()
    row_part = game_pos[1:]
    if not ('A' <= col_part <= 'Z'):
        raise ValueError(f"Invalid column: {col_part}")
    if not row_part.isdigit() or not (1 <= int(row_part) <= 26):
        raise ValueError(f"Invalid row: {row_part}")
    col = ord(col_part) - ord('A')
    row = int(row_part) - 1
    col += 0.5
    row += 0.5
    px = int((col / 26) * width)
    py = int((row / 26) * height)
    return px, py

# Convert map coordinates to pixel coordinates
def map_to_pixel(mx, my, width, height):
    px = int((mx / 26) * width)
    py = int(((my + 26) / 26) * height)
    return px, py

# A* algorithm implementation with multithreading
def a_star(start, goal, grid, grid_resolution, waypoints, result_queue, stop_event, visited_lock, visited):
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])  # Euclidean distance

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set and not stop_event.is_set():
        _, current = heapq.heappop(open_set)

        # Check if the current node has already been visited by another thread
        with visited_lock:
            if current in visited:
                continue
            visited.add(current)

        if current == goal:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            result_queue.put(path[::-1])  # Return reversed path
            stop_event.set()  # Signal other threads to stop
            return

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # 8-connected grid
            neighbor = (current[0] + dx, current[1] + dy)

            if neighbor[0] < 0 or neighbor[0] >= grid_resolution or neighbor[1] < 0 or neighbor[1] >= grid_resolution:
                continue  # Out of bounds

            # Allow waypoints to be traversable even if they are in a safe zone
            if grid[neighbor[1]][neighbor[0]] == 1 and neighbor not in waypoints:
                continue  # Blocked by obstacle

            tentative_g_score = g_score[current] + heuristic(current, neighbor)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Create a high-resolution grid
def create_high_resolution_grid(safe_areas, waypoints, width, height, grid_resolution=500):
    grid = np.zeros((grid_resolution, grid_resolution), dtype=np.uint8)  # Use NumPy for faster array operations
    cell_size = width / grid_resolution  # Size of each grid cell

    # Precompute waypoint grid positions
    waypoint_grids = set()
    for wp in waypoints:
        wx, wy = wp
        wx_grid = int(wx / (width / grid_resolution))
        wy_grid = int(wy / (height / grid_resolution))
        waypoint_grids.add((wx_grid, wy_grid))

    # Mark safe zones as blocked unless the current waypoint is inside them
    for island in safe_areas:
        try:
            if "x" not in island or "y" not in island or "safe_radius" not in island:
                raise KeyError("Missing 'x', 'y', or 'safe_radius'")

            cx, cy = map_to_pixel(island["x"], island["y"], width, height)
            radius = int(island["safe_radius"] * (width / 26))

            if radius <= 0:
                print(f"Warning: Island has invalid or zero 'safe_radius': {island}")
                continue

            # Check if the current waypoint is inside this safe zone
            waypoint_inside = False
            for wp in waypoints:
                wx, wy = wp
                distance = math.hypot(wx - cx, wy - cy)
                if distance < radius:
                    waypoint_inside = True
                    break

            # If no waypoint is inside the safe zone, mark it as blocked
            if not waypoint_inside:
                grid_x_start = max(0, int((cx - radius) / cell_size))
                grid_x_end = min(grid_resolution, int((cx + radius) / cell_size) + 1)
                grid_y_start = max(0, int((cy - radius) / cell_size))
                grid_y_end = min(grid_resolution, int((cy + radius) / cell_size) + 1)

                xx, yy = np.meshgrid(
                    np.arange(grid_x_start, grid_x_end),
                    np.arange(grid_y_start, grid_y_end)
                )
                grid_cells = np.column_stack((xx.ravel(), yy.ravel()))
                grid_centers = grid_cells * cell_size + cell_size / 2
                grid_distances = np.linalg.norm(grid_centers - np.array([cx, cy]), axis=1)
                blocked_cells = grid_cells[grid_distances < radius]
                for cell in blocked_cells:
                    grid[cell[1], cell[0]] = 1

        except KeyError as e:
            print(f"Warning: Island is missing key '{e}': {island}")
        except Exception as e:
            print(f"Error processing island: {island}. Error: {e}")

    return grid

# Check if a line between two points is collision-free
def is_line_collision_free(point1, point2, grid, grid_resolution):
    x1, y1 = point1
    x2, y2 = point2
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))
    for i in range(steps):
        x = int(x1 + i * dx / steps)
        y = int(y1 + i * dy / steps)
        if grid[y][x] == 1:
            return False
    return True

# Smooth the path using line-of-sight checks
def smooth_path(path, grid, grid_resolution, distance_threshold=5.0):
    if not path:
        return path

    smoothed_path = [path[0]]
    current_index = 0

    while current_index < len(path) - 1:
        next_index = len(path) - 1
        while next_index > current_index + 1:
            if is_line_collision_free(path[current_index], path[next_index], grid, grid_resolution):
                break
            next_index -= 1

        last_point = smoothed_path[-1]
        current_point = path[next_index]
        distance = math.hypot(current_point[0] - last_point[0], current_point[1] - last_point[1])

        # Check for duplicate points based on exact coordinates
        if distance >= distance_threshold and current_point not in smoothed_path:
            smoothed_path.append(path[next_index])

        current_index = next_index

    return smoothed_path



# Find a path from start to goal using A* with multithreading
def find_path(start, goal, grid, width, height, grid_resolution=500):
    """
    Find a path from start to goal using the precomputed grid.
    
    Args:
        start (tuple): Start coordinates (x, y).
        goal (tuple): Goal coordinates (x, y).
        grid (numpy.ndarray): Precomputed grid.
        width (int): Width of the map.
        height (int): Height of the map.
        grid_resolution (int): Resolution of the grid.
    
    Returns:
        list: List of (x, y) coordinates representing the path.
    """
    # Convert start and goal to grid coordinates
    start_grid = (int(start[0] / (width / grid_resolution)), int(start[1] / (height / grid_resolution)))
    goal_grid = (int(goal[0] / (width / grid_resolution)), int(goal[1] / (height / grid_resolution)))

    # Validate grid coordinates
    if (start_grid[0] < 0 or start_grid[0] >= grid_resolution or
        start_grid[1] < 0 or start_grid[1] >= grid_resolution or
        goal_grid[0] < 0 or goal_grid[0] >= grid_resolution or
        goal_grid[1] < 0 or goal_grid[1] >= grid_resolution):
        raise ValueError("Start or goal coordinates are out of grid bounds.")

    result_queue = Queue()
    stop_event = threading.Event()
    visited_lock = threading.Lock()
    visited = set()

    num_threads = 8
    threads = []
    for _ in range(num_threads):
        thread = threading.Thread(
            target=a_star,
            args=(start_grid, goal_grid, grid, grid_resolution, [], result_queue, stop_event, visited_lock, visited)
        )
        threads.append(thread)
        thread.start()

    path_grid = result_queue.get()
    stop_event.set()

    for thread in threads:
        thread.join()

    if not path_grid:
        raise ValueError("No path found")

    smoothed_path_grid = smooth_path(path_grid, grid, grid_resolution)

    path = [(x * (width / grid_resolution), y * (height / grid_resolution)) for x, y in smoothed_path_grid]
    return path

# Draw the final map with islands and the path, and overlay waypoint order numbers
def draw_map(image, safe_areas, path, waypoints, width, height, scale_factor=0.6):
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Draw safe areas
    for island in safe_areas:
        px, py = map_to_pixel(island["x"], island["y"], width, height)
        pradius = int(island["safe_radius"] * (width / 26))
        cv2.circle(color_image, (px, py), pradius, (0, 255, 0), 2)

    # Draw the path
    if path:
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            cv2.line(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

        # Draw red dots (path points)
        for idx, (x, y) in enumerate(path):
            cv2.circle(color_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            # Overlay the index (only on red dots)
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = str(idx + 1)  # Display the index number (starting from 1)
            text_size = cv2.getTextSize(text, font, 0.6, 2)[0]  # Adjust thickness for bold
            text_x = int(x - text_size[0] / 2)
            text_y = int(y - 10)
            
            cv2.putText(color_image, text, (text_x, text_y), font, 0.6, (0, 0, 255), 2, cv2.LINE_AA)  # Thickness = 2 for bold

    # Draw waypoints with cyan dots, no numbers
    for sx, sy in waypoints:
        cv2.circle(color_image, (int(sx), int(sy)), 7, (255, 255, 0), -1)

    # Resize the image to fit the screen
    resized_image = cv2.resize(color_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)

    # Display the resized image
    cv2.imshow("Map with Path", resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Function to save the grid to a file
def save_grid(grid,filename="precomputed_grid.npy"):
    # Save the grid to the file
    np.save(filename, grid)
    print(f"Grid saved to {filename}")

def load_grid(filename="precomputed_grid.npy"):
    # Check if the file exists
    if os.path.exists(filename):
        grid = np.load(filename)
        print(f"Grid loaded from {filename}")
        return grid
    else:
        print(f"No precomputed grid found at {filename}")
        return None

# Save the path to a file
def save_path_to_file(path, filename="path_output.txt"):
    with open(filename, "w") as f:
        for point in path:
            f.write(f"{point[0]}, {point[1]}\n")
    print(f"Path saved to {filename}")

# Function to remove duplicate points within a tolerance of �5%
def remove_duplicates(path, tolerance=0.05):
    if not path:
        return path
    
    filtered_path = [path[0]]  # Start with the first point
    
    for i in range(1, len(path)):
        prev_point = filtered_path[-1]
        curr_point = path[i]
        
        # Calculate the distance between the current point and the previous one
        distance = math.hypot(curr_point[0] - prev_point[0], curr_point[1] - prev_point[1])
        
        # Calculate the tolerance distance (5% of the previous point's distance)
        tolerance_distance = tolerance * distance
        
        # If the distance is greater than the tolerance distance, add the current point
        if distance > tolerance_distance:
            filtered_path.append(curr_point)
    
    return filtered_path

# Main program continuation
if __name__ == "__main__":
    if os.path.exists("path_output.txt"):
        os.remove("path_output.txt")

    image = cv2.imread("images/bw_map.png", cv2.IMREAD_GRAYSCALE)
    height, width = image.shape[:2]

    with open("island_positions.json", "r") as f:
        safe_areas = json.load(f)

    # Input waypoints dynamically
    print("Enter waypoints (e.g., A1 B3 C5):")
    raw_waypoints = input("Waypoints: ").strip().split()
    waypoints = [game_to_coords(wp, width, height) for wp in raw_waypoints]

    final_goal = waypoints[-1]  # Assuming the last waypoint is the goal
    waypoints.append(final_goal)

    # Precompute the grid during initialization
    grid_resolution = 5000  # High-resolution grid
    grid_filename = "precomputed_grid.npy"

    # Check if a precomputed grid exists
    grid = load_grid(grid_filename)

    # If no precomputed grid exists, compute it and save it
    if grid is None:
        print("Precomputing grid...")
        grid = create_high_resolution_grid(safe_areas, waypoints, width, height, grid_resolution)
        save_grid(grid, grid_filename)
        print("Grid precomputed and saved.")
    else:
        print("Using precomputed grid.")

    # Find the full path
    full_path = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]
        print(f"{i} - Finding path from {start} to {goal}...")

        # Find path using A* with the precomputed grid
        path = find_path(start, goal, grid, width, height, grid_resolution)

        if not path:
            print(f"No path found from {start} to {goal}.")
            continue

        full_path.extend(path)

    # Remove duplicates (within 5% tolerance)
    full_path = remove_duplicates(full_path, tolerance=0.05)

    # Output the path
    print("Path Coordinates (without duplicates):")
    for point in full_path:
        print(f"({point[0]}, {point[1]})")

    # Save the path to a file
    save_path_to_file(full_path)

    # Draw the final path
    draw_map(image, safe_areas, full_path, waypoints, width, height)
