# src/perception.py
# Simulates a LiDAR sensor using ray-casting.
# The agent fires NUM_SENSOR_RAYS rays in all directions.
# Each ray travels until it hits a wall, obstacle, or max range.
# This mimics how real robots perceive their environment.

import math
from config import (
    CELL_SIZE, NUM_SENSOR_RAYS, AGENT_SENSOR_RANGE,
    GRID_COLS, GRID_ROWS
)


class LiDARSensor:
    """
    Simulates a rotating LiDAR sensor attached to the agent.

    Ray-casting approach:
    - Fire rays at equal angular intervals around 360 degrees
    - Each ray steps cell by cell until it hits an obstacle or exceeds range
    - Returns a list of hit points (pixel coordinates) and hit distances
    """

    def __init__(self, environment):
        self.env = environment
        self.num_rays = NUM_SENSOR_RAYS
        self.max_range = AGENT_SENSOR_RANGE  # in grid cells
        self.last_scan = []    # List of (hit_px, hit_py, distance, hit_obstacle)

    def scan(self, agent_px, agent_py):
        """
        Perform a full LiDAR scan from the agent's position.

        Returns:
            scan_results: list of dicts with ray info
        """
        self.last_scan = []
        angle_step = 360.0 / self.num_rays

        # Get dynamic obstacle cells for this frame
        dynamic_cells = self.env.get_dynamic_obstacle_cells()

        for i in range(self.num_rays):
            angle_deg = i * angle_step
            angle_rad = math.radians(angle_deg)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            hit = False
            hit_px = agent_px + dx * self.max_range * CELL_SIZE
            hit_py = agent_py + dy * self.max_range * CELL_SIZE
            hit_dist = self.max_range  # Default = max range (no hit)
            hit_type = 'none'  # 'static', 'dynamic', 'none'

            # Step along the ray, checking each cell
            for step in range(1, self.max_range + 1):
                check_px = agent_px + dx * step * CELL_SIZE
                check_py = agent_py + dy * step * CELL_SIZE

                check_col = int(check_px // CELL_SIZE)
                check_row = int(check_py // CELL_SIZE)

                # Out of bounds check
                if not (0 <= check_col < GRID_COLS and 0 <= check_row < GRID_ROWS):
                    hit = True
                    hit_px, hit_py = check_px, check_py
                    hit_dist = step
                    hit_type = 'boundary'
                    break

                # Static obstacle check
                if self.env.is_cell_blocked(check_col, check_row):
                    hit = True
                    hit_px = check_col * CELL_SIZE + CELL_SIZE // 2
                    hit_py = check_row * CELL_SIZE + CELL_SIZE // 2
                    hit_dist = step
                    hit_type = 'static'
                    break

                # Dynamic obstacle check
                if (check_col, check_row) in dynamic_cells:
                    hit = True
                    hit_px = check_col * CELL_SIZE + CELL_SIZE // 2
                    hit_py = check_row * CELL_SIZE + CELL_SIZE // 2
                    hit_dist = step
                    hit_type = 'dynamic'
                    break

            self.last_scan.append({
                'angle': angle_deg,
                'hit_px': hit_px,
                'hit_py': hit_py,
                'distance': hit_dist,
                'hit': hit,
                'hit_type': hit_type
            })

        return self.last_scan

    def get_obstacle_cells_in_range(self, agent_col, agent_row):
        """
        Return a set of grid cells detected as obstacles by sensor.
        Used by occupancy grid builder.
        """
        detected = set()
        for ray in self.last_scan:
            if ray['hit']:
                c = int(ray['hit_px'] // CELL_SIZE)
                r = int(ray['hit_py'] // CELL_SIZE)
                if 0 <= c < GRID_COLS and 0 <= r < GRID_ROWS:
                    detected.add((c, r))
        return detected