# tests/test_path_planner.py
# Unit tests for the A* path planner.
# Run with: python -m pytest tests/

import numpy as np
import pytest
from src.path_planner import PathPlanner


def make_empty_grid(rows=10, cols=10):
    """Create a fully free grid."""
    return np.zeros((rows, cols), dtype=int)


def make_walled_grid():
    """Grid with a wall forcing a detour."""
    grid = make_empty_grid(10, 10)
    # Vertical wall at column 5, rows 0-7
    for r in range(8):
        grid[r][5] = 1
    return grid


class TestPathPlanner:

    def setup_method(self):
        self.planner = PathPlanner()

    def test_finds_path_empty_grid(self):
        grid = make_empty_grid()
        path = self.planner.find_path((0,0), (9,9), grid)
        assert len(path) > 0, "Should find a path in empty grid"
        assert path[0] == (0,0), "Path should start at start"
        assert path[-1] == (9,9), "Path should end at goal"

    def test_no_path_when_goal_blocked(self):
        grid = make_empty_grid()
        grid[9][9] = 1   # Block the goal
        path = self.planner.find_path((0,0), (9,9), grid)
        assert path == [], "Should return empty path if goal blocked"

    def test_detour_around_wall(self):
        grid = make_walled_grid()
        path = self.planner.find_path((0,0), (9,9), grid)
        assert len(path) > 0, "Should find detour around wall"
        # Path should not go through the wall
        for (col, row) in path:
            assert grid[row][col] == 0, "Path should not cross obstacles"

    def test_same_start_goal(self):
        grid = make_empty_grid()
        path = self.planner.find_path((5,5), (5,5), grid)
        # Single point path is acceptable
        assert len(path) >= 1

    def test_adjacent_cells(self):
        grid = make_empty_grid()
        path = self.planner.find_path((3,3), (4,3), grid)
        assert len(path) == 2, "Adjacent cells should give 2-point path"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])