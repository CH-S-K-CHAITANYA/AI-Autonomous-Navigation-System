# simulation/environment.py
# Creates and manages the simulation world:
# - Grid map
# - Static obstacles (walls, blocks)
# - Dynamic obstacles (moving entities)
# - Start and goal positions

import random
import numpy as np
from config import (
    GRID_COLS, GRID_ROWS, CELL_SIZE,
    NUM_STATIC_OBSTACLES, NUM_DYNAMIC_OBSTACLES,
    DYNAMIC_OBSTACLE_SPEED
)


class DynamicObstacle:
    """A moving obstacle that patrols between two points."""

    def __init__(self, grid_x, grid_y, direction='horizontal'):
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.direction = direction
        self.speed = DYNAMIC_OBSTACLE_SPEED
        self.move_counter = 0.0
        self.patrol_range = 3          # Cells to patrol
        self.patrol_origin_x = grid_x
        self.patrol_origin_y = grid_y
        self.patrol_dir = 1            # +1 or -1

    def update(self):
        """Move the obstacle back and forth."""
        self.move_counter += self.speed * 0.05
        if self.direction == 'horizontal':
            offset = int(self.patrol_range * np.sin(self.move_counter))
            self.grid_x = self.patrol_origin_x + offset
        else:
            offset = int(self.patrol_range * np.sin(self.move_counter))
            self.grid_y = self.patrol_origin_y + offset

        # Clamp to grid bounds
        self.grid_x = max(1, min(GRID_COLS - 2, self.grid_x))
        self.grid_y = max(1, min(GRID_ROWS - 2, self.grid_y))

    def get_pixel_pos(self):
        """Convert grid coordinates to pixel coordinates (center of cell)."""
        px = self.grid_x * CELL_SIZE + CELL_SIZE // 2
        py = self.grid_y * CELL_SIZE + CELL_SIZE // 2
        return px, py


class Environment:
    """
    Manages the simulation world.
    The grid uses 0 = free, 1 = static obstacle.
    Dynamic obstacles are tracked separately (they move).
    """

    def __init__(self, seed=42):
        random.seed(seed)
        np.random.seed(seed)

        # 2D grid: 0 = passable, 1 = wall/blocked
        self.grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
        self.static_obstacles = []    # List of (col, row) tuples
        self.dynamic_obstacles = []   # List of DynamicObstacle objects
        self.start = None             # (col, row)
        self.goal = None              # (col, row)

        self._build_border_walls()
        self._place_static_obstacles()
        self._place_dynamic_obstacles()
        self._set_start_goal()

    def _build_border_walls(self):
        """Mark the grid border as walls."""
        # Top and bottom rows
        self.grid[0, :]          = 1
        self.grid[GRID_ROWS-1, :] = 1
        # Left and right columns
        self.grid[:, 0]          = 1
        self.grid[:, GRID_COLS-1] = 1

        # Record border cells
        for c in range(GRID_COLS):
            self.static_obstacles.append((c, 0))
            self.static_obstacles.append((c, GRID_ROWS-1))
        for r in range(1, GRID_ROWS-1):
            self.static_obstacles.append((0, r))
            self.static_obstacles.append((GRID_COLS-1, r))

    def _place_static_obstacles(self):
        """Randomly place L-shaped and single-cell obstacles."""
        placed = 0
        attempts = 0

        while placed < NUM_STATIC_OBSTACLES and attempts < 500:
            attempts += 1
            col = random.randint(2, GRID_COLS - 3)
            row = random.randint(2, GRID_ROWS - 3)

            # Decide shape: single cell or small L-shape
            shape_type = random.choice(['single', 'single', 'L', 'line'])

            cells = []
            if shape_type == 'single':
                cells = [(col, row)]
            elif shape_type == 'L':
                cells = [(col, row), (col+1, row), (col, row+1)]
            elif shape_type == 'line':
                length = random.randint(2, 3)
                if random.random() > 0.5:
                    cells = [(col+i, row) for i in range(length)]
                else:
                    cells = [(col, row+i) for i in range(length)]

            # Check all cells are free and within bounds
            valid = True
            for (c, r) in cells:
                if not (1 <= c < GRID_COLS-1 and 1 <= r < GRID_ROWS-1):
                    valid = False
                    break
                if self.grid[r][c] == 1:
                    valid = False
                    break

            if valid:
                for (c, r) in cells:
                    self.grid[r][c] = 1
                    self.static_obstacles.append((c, r))
                placed += 1

    def _place_dynamic_obstacles(self):
        """Place moving obstacles in open areas."""
        placed = 0
        attempts = 0
        while placed < NUM_DYNAMIC_OBSTACLES and attempts < 200:
            attempts += 1
            col = random.randint(3, GRID_COLS - 4)
            row = random.randint(3, GRID_ROWS - 4)

            if self.grid[row][col] == 0:
                direction = random.choice(['horizontal', 'vertical'])
                obs = DynamicObstacle(col, row, direction)
                self.dynamic_obstacles.append(obs)
                placed += 1

    def _set_start_goal(self):
        """Find valid, well-separated start and goal positions."""
        margin = 3  # Stay away from borders

        def random_free_cell():
            while True:
                c = random.randint(margin, GRID_COLS - margin)
                r = random.randint(margin, GRID_ROWS - margin)
                if self.grid[r][c] == 0:
                    return (c, r)

        self.start = random_free_cell()

        # Goal must be far from start (at least 15 cells Manhattan distance)
        for _ in range(1000):
            candidate = random_free_cell()
            dist = abs(candidate[0] - self.start[0]) + abs(candidate[1] - self.start[1])
            if dist >= 15:
                self.goal = candidate
                break

        if self.goal is None:
            self.goal = random_free_cell()

    def update(self):
        """Update dynamic obstacles each frame."""
        for obs in self.dynamic_obstacles:
            obs.update()

    def get_dynamic_obstacle_cells(self):
        """Return current grid cells occupied by dynamic obstacles."""
        cells = set()
        for obs in self.dynamic_obstacles:
            cells.add((obs.grid_x, obs.grid_y))
            # Also mark adjacent cells as risky
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = obs.grid_x + dx, obs.grid_y + dy
                if 0 <= nx < GRID_COLS and 0 <= ny < GRID_ROWS:
                    cells.add((nx, ny))
        return cells

    def is_cell_blocked(self, col, row):
        """Check if a cell is statically blocked."""
        if not (0 <= col < GRID_COLS and 0 <= row < GRID_ROWS):
            return True
        return self.grid[row][col] == 1