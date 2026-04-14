# src/path_planner.py
# Implementation of the A* (A-Star) pathfinding algorithm.
#
# A* is the gold standard for grid-based navigation in robotics.
# It finds the shortest path from start to goal while avoiding obstacles.
# It uses a heuristic (estimated distance to goal) to search efficiently.
#
# f(n) = g(n) + h(n)
# g(n) = actual cost from start to n
# h(n) = heuristic estimated cost from n to goal (Manhattan or Euclidean)

import heapq
import math
from config import GRID_COLS, GRID_ROWS, ALLOW_DIAGONAL


class PathPlanner:
    """A* pathfinding for the autonomous navigation system."""

    def __init__(self):
        self.last_path = []
        self.nodes_explored = 0

    def heuristic(self, a, b):
        """
        Euclidean distance heuristic.
        Works well when diagonal movement is allowed.
        """
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, col, row, planning_grid):
        """
        Return valid neighboring cells.
        Supports 4-directional or 8-directional movement.
        """
        neighbors = []

        # Cardinal directions (always included)
        directions_4 = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        # Diagonal directions (included if ALLOW_DIAGONAL)
        directions_diagonal = [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        directions = directions_4
        if ALLOW_DIAGONAL:
            directions = directions_4 + directions_diagonal

        for dc, dr in directions:
            nc, nr = col + dc, row + dr

            # Bounds check
            if not (0 <= nc < GRID_COLS and 0 <= nr < GRID_ROWS):
                continue

            # Obstacle check
            if planning_grid[nr][nc] != 0:
                continue

            # Diagonal safety: prevent cutting through wall corners
            if ALLOW_DIAGONAL and dc != 0 and dr != 0:
                if planning_grid[row][nc] != 0 or planning_grid[nr][col] != 0:
                    continue  # Corner cut blocked

            # Cost: diagonal moves cost sqrt(2), cardinal cost 1
            cost = math.sqrt(2) if (dc != 0 and dr != 0) else 1.0
            neighbors.append((nc, nr, cost))

        return neighbors

    def find_path(self, start, goal, planning_grid):
        """
        Run A* from start to goal.

        Args:
            start: (col, row) tuple
            goal:  (col, row) tuple
            planning_grid: 2D numpy array (0=free, 1=blocked)

        Returns:
            path: list of (col, row) tuples from start to goal
                  Returns empty list if no path found.
        """
        self.nodes_explored = 0

        # Validate start and goal
        if planning_grid[start[1]][start[0]] != 0:
            return []  # Start is blocked
        if planning_grid[goal[1]][goal[0]] != 0:
            return []  # Goal is blocked

        # Priority queue: (f_score, g_score, col, row)
        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, start[0], start[1]))

        # Track where we came from
        came_from = {}
        # g_score[node] = cheapest known cost from start to node
        g_score = {start: 0.0}
        # Track visited nodes
        closed_set = set()

        while open_set:
            _, g_current, col, row = heapq.heappop(open_set)
            current = (col, row)

            if current in closed_set:
                continue
            closed_set.add(current)
            self.nodes_explored += 1

            # Goal reached — reconstruct path
            if current == goal:
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                self.last_path = path
                return path

            # Explore neighbors
            for nc, nr, move_cost in self.get_neighbors(col, row, planning_grid):
                neighbor = (nc, nr)
                if neighbor in closed_set:
                    continue

                tentative_g = g_current + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    f = tentative_g + h
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f, tentative_g, nc, nr))

        # No path found
        self.last_path = []
        return []

    def smooth_path(self, path):
        """
        Simple path smoothing using string pulling.
        Removes unnecessary waypoints on straight lines.
        Reduces zigzag motion.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Try to skip as many intermediate points as possible
            j = len(path) - 1
            while j > i + 1:
                # Check if direct line from path[i] to path[j] is clear
                # (simplified: check just a few intermediate points)
                if self._line_of_sight(path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j

        return smoothed

    def _line_of_sight(self, a, b):
        """Check if there's a clear line between two grid points (simplified)."""
        # For our simulation, we just allow smoothing between adjacent waypoints
        # A real implementation would ray-march through the grid
        dist = abs(a[0] - b[0]) + abs(a[1] - b[1])
        return dist <= 3  # Only smooth if within 3 cells