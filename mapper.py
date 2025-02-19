import cv2
import numpy as np
import json

# Load the B/W island map
image = cv2.imread("images/bw_map.png", cv2.IMREAD_GRAYSCALE)

# Convert to binary (black = islands, white = water)
_, binary = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)

# Find contours (island shapes)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Get image dimensions
height, width = binary.shape

# Scaling factors (X: 0 to 26, Y: 0 to -26)
x_min, x_max = 0, 26
y_min, y_max = 0, -26

# Convert map coordinates to pixel coordinates
def pixel_coords(mx, my):
    px = (mx - x_min) * (width / (x_max - x_min))
    py = (my - y_min) * (height / (y_max - y_min))
    return int(px), int(py)

# Convert pixel coordinates to map coordinates
def map_coords(px, py):
    mx = (px / width) * (x_max - x_min) + x_min
    my = (py / height) * (y_max - y_min) + y_min
    return mx, my

island_data = []

# Process each island
for contour in contours:
    # Find the minimum enclosing circle
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    
    # Convert to map coordinates
    mapped_x, mapped_y = map_coords(cx, cy)
    
    # Convert radius from pixels to map units
    mapped_radius = (radius / width) * (x_max - x_min)+(100/600)
    
    island_data.append({
        "x": round(mapped_x, 2),
        "y": round(mapped_y, 2),
        "safe_radius": round(mapped_radius, 2)  # Dynamic radius
    })

# Save to JSON
with open("island_positions.json", "w") as f:
    json.dump(island_data, f, indent=4)

# Print results
print(json.dumps(island_data, indent=4))

# Load the original color image for visualization
color_image = cv2.imread("images/bw_map.png")

# Draw the circles on the original image
for island in island_data:
    # Convert map coordinates back to pixel coordinates
    px, py = pixel_coords(island["x"], island["y"])
    pradius = int(island["safe_radius"] * (width / (x_max - x_min)))
    
    # Draw the circle
    cv2.circle(color_image, (px, py), pradius, (0, 255, 0), 2)

# Display the image with circles
cv2.imshow("Islands with Circles", color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()