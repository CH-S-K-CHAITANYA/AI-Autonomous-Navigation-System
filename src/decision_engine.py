# src/decision_engine.py
# Decision engine: determines what the agent should do each frame.
# Uses a state machine pattern — very common in robotics systems.
#
# States:
#   INITIALIZING  → Computing first path
#   MOVING        → Following waypoints toward goal
#   REPLANNING    → Obstacle detected, computing new path
#   WAITING       → Brief pause after replanning
#   ARRIVED       → Goal reached
#   STUCK         → Cannot find any path (edge case)

from config import REPLAN_COOLDOWN


class AgentState:
    INITIALIZING = "INITIALIZING"
    MOVING       = "MOVING"
    REPLANNING   = "REPLANNING"
    WAITING      = "WAITING"
    ARRIVED      = "ARRIVED"
    STUCK        = "STUCK"


class DecisionEngine:
    """
    Monitors the agent's situation and decides when to act.
    
    Key decisions:
    1. Is the current path blocked? → Trigger replanning
    2. Has the agent arrived? → Stop
    3. Is the agent stuck? → Alert
    """

    def __init__(self, environment, path_planner):
        self.env = environment
        self.planner = path_planner
        self.state = AgentState.INITIALIZING
        self.replan_cooldown = 0   # Frames until next replan is allowed
        self.stuck_counter = 0
        self.status_message = "Initializing..."

    def update(self, agent, occupancy_grid, goal):
        """
        Main decision function — called every frame.
        Returns: (new_state, should_replan)
        """
        self.replan_cooldown = max(0, self.replan_cooldown - 1)

        # ── State: ARRIVED ──────────────────────────────────────
        if agent.arrived:
            self.state = AgentState.ARRIVED
            self.status_message = "✓ Goal Reached!"
            return self.state, False

        # ── State: INITIALIZING ─────────────────────────────────
        if self.state == AgentState.INITIALIZING:
            self.status_message = "Planning initial path..."
            return self.state, True   # Signal: compute path now

        # ── State: MOVING ────────────────────────────────────────
        if self.state in [AgentState.MOVING, AgentState.WAITING]:
            # Check if upcoming waypoints are still clear
            upcoming = agent.get_remaining_waypoints()
            planning_grid = occupancy_grid.get_grid_for_planning()
            path_blocked = False

            # Check the next 3 waypoints for blockage
            for wp in upcoming[:3]:
                c, r = wp
                if planning_grid[r][c] != 0:
                    path_blocked = True
                    break

            if path_blocked and self.replan_cooldown == 0:
                self.state = AgentState.REPLANNING
                self.status_message = "⚠ Obstacle! Replanning..."
                return self.state, True

            self.state = AgentState.MOVING
            if not agent.is_moving:
                # Path finished but goal not marked arrived — recheck
                self.status_message = "Recalculating..."
                return self.state, True

            self.status_message = f"Moving → Goal ({goal[0]},{goal[1]})"
            return self.state, False

        # ── State: REPLANNING ────────────────────────────────────
        if self.state == AgentState.REPLANNING:
            self.replan_cooldown = REPLAN_COOLDOWN
            self.state = AgentState.WAITING
            return self.state, True  # Trigger replan computation

        return self.state, False

    def on_path_computed(self, path, agent):
        """Called after the planner returns a result."""
        if path:
            self.stuck_counter = 0
            self.state = AgentState.MOVING
        else:
            self.stuck_counter += 1
            if self.stuck_counter >= 3:
                self.state = AgentState.STUCK
                self.status_message = "✗ No path found — STUCK"
            else:
                self.state = AgentState.WAITING
                self.status_message = "Path blocked. Waiting..."