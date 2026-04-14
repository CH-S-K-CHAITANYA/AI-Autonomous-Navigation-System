# src/agent.py
# The autonomous vehicle agent.
# Tracks position, orientation, speed, and movement history.

import math
from config import CELL_SIZE, AGENT_SPEED, AGENT_RADIUS


class Agent:
    """
    Represents the autonomous vehicle in the simulation.

    Position is tracked in pixel coordinates (float precision)
    for smooth movement. Grid coordinates are derived from pixel pos.
    """

    def __init__(self, start_grid_col, start_grid_row):
        # Convert grid start position to pixel coordinates (center of cell)
        self.px = start_grid_col * CELL_SIZE + CELL_SIZE // 2
        self.py = start_grid_row * CELL_SIZE + CELL_SIZE // 2
        self.angle = 0.0            # Facing angle in degrees (0 = right)
        self.speed = AGENT_SPEED
        self.radius = AGENT_RADIUS

        # Navigation state
        self.waypoints = []         # Current path as list of (col, row)
        self.current_waypoint_idx = 0
        self.is_moving = False
        self.arrived = False

        # Statistics
        self.total_distance = 0.0
        self.steps_taken = 0
        self.replan_count = 0
        self.trail = []             # History of pixel positions for drawing

    def get_grid_pos(self):
        """Return current grid cell (col, row)."""
        col = int(self.px // CELL_SIZE)
        row = int(self.py // CELL_SIZE)
        return col, row

    def set_path(self, path, is_replan=False):
        """
        Set a new path for the agent to follow.
        path: list of (col, row) tuples from planner
        """
        if path and len(path) > 1:
            self.waypoints = path[1:]  # Skip first cell (current position)
            self.current_waypoint_idx = 0
            self.is_moving = True
            self.arrived = False
            if is_replan:
                self.replan_count += 1

    def update(self):
        """
        Move the agent one step toward the current waypoint.
        Called every frame.
        """
        if not self.is_moving or not self.waypoints:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.is_moving = False
            self.arrived = True
            return

        # Target waypoint in pixel coordinates
        target_col, target_row = self.waypoints[self.current_waypoint_idx]
        target_px = target_col * CELL_SIZE + CELL_SIZE // 2
        target_py = target_row * CELL_SIZE + CELL_SIZE // 2

        # Direction to target
        dx = target_px - self.px
        dy = target_py - self.py
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.speed:
            # Snap to waypoint and advance
            self.px = target_px
            self.py = target_py
            self.total_distance += distance
            self.current_waypoint_idx += 1
            self.steps_taken += 1
        else:
            # Move toward target
            self.angle = math.degrees(math.atan2(dy, dx))
            move_x = (dx / distance) * self.speed
            move_y = (dy / distance) * self.speed
            old_px, old_py = self.px, self.py
            self.px += move_x
            self.py += move_y
            self.total_distance += math.sqrt(move_x**2 + move_y**2)

        # Record trail (every 5 frames to avoid too many points)
        if self.steps_taken % 3 == 0:
            self.trail.append((int(self.px), int(self.py)))
            if len(self.trail) > 200:   # Keep last 200 trail points
                self.trail.pop(0)

    def get_remaining_waypoints(self):
        """Return waypoints not yet visited."""
        if self.current_waypoint_idx < len(self.waypoints):
            return self.waypoints[self.current_waypoint_idx:]
        return []