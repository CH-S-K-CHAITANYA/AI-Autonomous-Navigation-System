# src/occupancy_grid.py
# Builds a real-time occupancy grid from sensor readings.
# 0 = free, 1 = occupied (static), 2 = occupied (dynamic)
# This is what the path planner uses — not the raw environment.

import numpy as np
from config import GRID_COLS, GRID_ROWS


class OccupancyGrid:
    """
    Maintains a probabilistic view of the world as seen by the agent's sensors.

    In a real system, this would use Bayesian updates.
    Here we use a simplified version: sensor hits → mark as occupied.
    Dynamic obstacles are marked with a different value so the planner
    can weight them differently.
    """

    def __init__(self, environment):
        self.env = environment
        self.grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
        # Initialize with known static obstacles from environment
        self.grid = environment.grid.copy()

    def update(self, detected_static, dynamic_cells):
        """
        Update the occupancy grid with current sensor readings.

        detected_static: set of (col, row) detected by LiDAR as static
        dynamic_cells:   set of (col, row) of dynamic obstacles
        """
        # Reset dynamic markings (they move, so refresh each frame)
        self.grid[self.grid == 2] = 0

        # Mark static detections (these persist — real LiDAR builds a map)
        for (c, r) in detected_static:
            if 0 <= c < GRID_COLS and 0 <= r < GRID_ROWS:
                if self.env.is_cell_blocked(c, r):
                    self.grid[r][c] = 1

        # Mark dynamic obstacles
        for (c, r) in dynamic_cells:
            if 0 <= c < GRID_COLS and 0 <= r < GRID_ROWS:
                if self.grid[r][c] == 0:  # Don't overwrite static walls
                    self.grid[r][c] = 2

    def is_navigable(self, col, row):
        """Return True if cell is safe to navigate into."""
        if not (0 <= col < GRID_COLS and 0 <= row < GRID_ROWS):
            return False
        return self.grid[row][col] == 0

    def get_grid_for_planning(self):
        """
        Return a planning grid where 0 = free, 1 = blocked.
        Dynamic obstacles (2) are treated as blocked for planning.
        """
        planning_grid = (self.grid > 0).astype(int)
        return planning_grid