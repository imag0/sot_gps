import math
import os
import time

print("Pee Pee Poo Poo on standby...")
while not os.path.exists("path_output.txt"):
    time.sleep(1)
print("main.py has finished. Now running pipi.py...")
# Function to calculate the bearing between two points (x1, y1) and (x2, y2)
def calculate_bearing(x1, y1, x2, y2):
    # Calculate the differences in x and y
    delta_x = x2 - x1
    delta_y = y2 - y1

    # Calculate the angle in radians using atan2
    angle_radians = math.atan2(delta_y, delta_x)

    # Convert the angle to degrees
    angle_degrees = math.degrees(angle_radians)

    # Normalize the angle to the 0-360 range
    bearing = (angle_degrees + 360) % 360

    return bearing

# Function to read coordinates from a file and calculate the bearing
def read_coordinates_and_calculate_bearing(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

        coordinates = []
        for line in lines:
            # Replace the first comma in the line with a space
            line = line.replace(',', ' ', 1).strip()
            parts = line.split()
            if len(parts) == 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    coordinates.append((x, y))
                except ValueError:
                    print(f"Skipping invalid line: {line}")

        # Calculate the bearing for each pair of waypoints
        for i in range(len(coordinates) - 1):
            x1, y1 = coordinates[i]
            x2, y2 = coordinates[i + 1]
            bearing = calculate_bearing(x1, y1, x2, y2)+90
            if bearing > 360: bearing = bearing-360
            print(f"Bearing from waypoint {i+1} to {i+2}: {bearing:.2f}°")

file_path = 'path_output.txt'
read_coordinates_and_calculate_bearing(file_path)