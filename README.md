# Sea of Thieves GPS navigation with collision avoidance
## Could also be used for different tasks after tweaking
### GitHub Repository Description for the Bearing Calculator App

---

**Bearing Calculator**  
A simple Python application to calculate the bearing (angle) between two points on a 2D Cartesian plane. This tool is designed for maps or grids where `(0, 0)` represents the top-left corner and `(1200, 1200)` represents the bottom-right corner. It reads coordinates from a file, computes the bearing between consecutive points, and outputs the results in degrees.

---

### Features:
- **Bearing Calculation**: Computes the angle between two points relative to the positive x-axis using the `atan2` function.
- **Route Plotting**: Calculates the best (shortest) route between given waypoints.

---

### How It Works:
1. **Input File**: Provide a text file (`path_output.txt`) with coordinates in the format:
   ```
   x1, y1
   x2, y2
   x3, y3
   ```
2. **Bearing Calculation**: The app calculates the bearing between consecutive points using the formula:
   ```
   bearing = atan2(delta_y, delta_x)
   ```
   where `delta_x = x2 - x1` and `delta_y = y2 - y1`.
3. **Output**: The app prints the bearing for each pair of points in degrees.

---

### Requirements:
- Python 3.x
- opencv-python
- numpy

---

### Installation:
1. Clone the repository:
   ```bash
   git clone https://github.com/imag0/sot_gps.git
   ```
2. Navigate to the project directory:
   ```bash
   cd sot_gps
   ```
3. Run the starter.bat
   
---

### Contributing:
Contributions are welcome! If you find a bug or have a feature request, please open an issue or submit a pull request.

---

### License:
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

### Author:
imag0


https://github.com/imag0
