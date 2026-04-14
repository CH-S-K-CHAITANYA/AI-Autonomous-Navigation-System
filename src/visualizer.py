# src/visualizer.py
# Handles all Pygame rendering.
# Draws: grid, obstacles, path, agent, sensors, trails, dashboard.

import pygame
import math
from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, SIM_WIDTH, DASHBOARD_WIDTH,
    CELL_SIZE, GRID_COLS, GRID_ROWS,
    BLACK, WHITE, GRAY, LIGHT_GRAY, DARK_GRAY, DARK_GREEN,
    RED, GREEN, BLUE, CYAN, YELLOW, ORANGE, PURPLE, PATH_COLOR,
    AGENT_RADIUS
)
from src.decision_engine import AgentState


class Visualizer:
    """Renders the complete simulation to the Pygame window."""

    def __init__(self, screen):
        self.screen = screen
        pygame.font.init()
        self.font_large  = pygame.font.SysFont("monospace", 18, bold=True)
        self.font_medium = pygame.font.SysFont("monospace", 14)
        self.font_small  = pygame.font.SysFont("monospace", 11)
        self.frame_count = 0

        # Pre-create semi-transparent surface for sensor overlay
        self.sensor_surf = pygame.Surface((SIM_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

    def render(self, env, agent, occupancy_grid, path, sensor_scan,
               decision_state, status_message, stats):
        """Main render function — draws everything for one frame."""
        self.frame_count += 1
        self.screen.fill(DARK_GRAY)

        # ── Draw simulation panel (left) ────────────────────────
        self._draw_grid(occupancy_grid)
        self._draw_path(path)
        self._draw_trail(agent)
        self._draw_sensor_rays(sensor_scan, agent)
        self._draw_dynamic_obstacles(env)
        self._draw_start_goal(env)
        self._draw_agent(agent, decision_state)
        self._draw_sim_border()

        # ── Draw dashboard panel (right) ────────────────────────
        self._draw_dashboard(agent, decision_state, status_message, stats)

    def _draw_grid(self, occupancy_grid):
        """Draw the occupancy grid — cells colored by type."""
        grid = occupancy_grid.grid
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                x = col * CELL_SIZE
                y = row * CELL_SIZE
                rect = pygame.Rect(x, y, CELL_SIZE - 1, CELL_SIZE - 1)

                val = grid[row][col]
                if val == 0:
                    # Free cell — subtle grid lines
                    pygame.draw.rect(self.screen, (30, 35, 30), rect)
                elif val == 1:
                    # Static obstacle — dark gray block
                    pygame.draw.rect(self.screen, (80, 80, 80), rect)
                    # 3D shading effect
                    pygame.draw.rect(self.screen, (100, 100, 100),
                                     pygame.Rect(x, y, CELL_SIZE-1, 3))
                    pygame.draw.rect(self.screen, (60, 60, 60),
                                     pygame.Rect(x, y+CELL_SIZE-4, CELL_SIZE-1, 3))
                elif val == 2:
                    # Dynamic obstacle — orange
                    pygame.draw.rect(self.screen, (100, 50, 0), rect)

        # Draw thin grid lines
        for col in range(GRID_COLS + 1):
            pygame.draw.line(self.screen, (40, 45, 40),
                             (col * CELL_SIZE, 0),
                             (col * CELL_SIZE, WINDOW_HEIGHT))
        for row in range(GRID_ROWS + 1):
            pygame.draw.line(self.screen, (40, 45, 40),
                             (0, row * CELL_SIZE),
                             (SIM_WIDTH, row * CELL_SIZE))

    def _draw_path(self, path):
        """Draw the planned A* path as connected circles."""
        if len(path) < 2:
            return

        # Draw path line
        pixel_path = []
        for (col, row) in path:
            px = col * CELL_SIZE + CELL_SIZE // 2
            py = row * CELL_SIZE + CELL_SIZE // 2
            pixel_path.append((px, py))

        if len(pixel_path) >= 2:
            pygame.draw.lines(self.screen, PATH_COLOR, False, pixel_path, 2)

        # Draw waypoint dots
        for i, (px, py) in enumerate(pixel_path):
            size = 4 if i > 0 and i < len(pixel_path)-1 else 6
            color = PATH_COLOR if i > 0 else GREEN
            pygame.draw.circle(self.screen, color, (px, py), size)

    def _draw_trail(self, agent):
        """Draw the agent's movement history as a fading trail."""
        trail = agent.trail
        for i, (px, py) in enumerate(trail):
            alpha = int(255 * i / max(len(trail), 1))
            r = max(0, CYAN[0] - (255 - alpha))
            g = max(0, CYAN[1] - (255 - alpha) // 2)
            b = CYAN[2]
            color = (min(255, r), min(255, g), b)
            pygame.draw.circle(self.screen, color, (px, py), 2)

    def _draw_sensor_rays(self, sensor_scan, agent):
        """Draw simulated LiDAR rays from agent."""
        if not sensor_scan:
            return

        agent_px = int(agent.px)
        agent_py = int(agent.py)

        for ray in sensor_scan:
            hit_px = int(ray['hit_px'])
            hit_py = int(ray['hit_py'])
            hit = ray['hit']
            hit_type = ray['hit_type']

            # Ray line: yellow for free, red for hit
            ray_color = (60, 60, 20) if not hit else (80, 30, 30)
            pygame.draw.line(self.screen, ray_color,
                             (agent_px, agent_py), (hit_px, hit_py), 1)

            # Hit point marker
            if hit:
                dot_color = RED if hit_type == 'static' else ORANGE
                pygame.draw.circle(self.screen, dot_color, (hit_px, hit_py), 3)

    def _draw_dynamic_obstacles(self, env):
        """Draw moving obstacles with animated effect."""
        for obs in env.dynamic_obstacles:
            px, py = obs.get_pixel_pos()
            size = CELL_SIZE // 2 - 2

            # Pulsing orange square
            pulse = abs(math.sin(self.frame_count * 0.1)) * 0.3 + 0.7
            r = int(220 * pulse)
            g = int(100 * pulse)
            pygame.draw.rect(self.screen, (r, g, 0),
                             pygame.Rect(px - size, py - size,
                                        size*2, size*2))
            pygame.draw.rect(self.screen, YELLOW,
                             pygame.Rect(px - size, py - size,
                                        size*2, size*2), 2)

            # Direction arrow
            label = self.font_small.render("DYN", True, WHITE)
            self.screen.blit(label, (px - 12, py - 6))

    def _draw_start_goal(self, env):
        """Draw start (green flag) and goal (red flag)."""
        # Start
        sc, sr = env.start
        sx = sc * CELL_SIZE + CELL_SIZE // 2
        sy = sr * CELL_SIZE + CELL_SIZE // 2
        pygame.draw.circle(self.screen, GREEN, (sx, sy), CELL_SIZE // 2 - 2)
        label = self.font_small.render("S", True, BLACK)
        self.screen.blit(label, (sx - 5, sy - 6))

        # Goal
        gc, gr = env.goal
        gx = gc * CELL_SIZE + CELL_SIZE // 2
        gy = gr * CELL_SIZE + CELL_SIZE // 2

        # Flashing goal marker
        pulse = int(abs(math.sin(self.frame_count * 0.08)) * 255)
        goal_color = (255, pulse, pulse)
        pygame.draw.circle(self.screen, goal_color, (gx, gy), CELL_SIZE // 2 - 2)
        label = self.font_small.render("G", True, BLACK)
        self.screen.blit(label, (gx - 5, gy - 6))

    def _draw_agent(self, agent, state):
        """Draw the autonomous vehicle agent."""
        ax = int(agent.px)
        ay = int(agent.py)
        radius = agent.radius

        # Body color depends on state
        state_colors = {
            AgentState.MOVING:      BLUE,
            AgentState.REPLANNING:  ORANGE,
            AgentState.WAITING:     YELLOW,
            AgentState.ARRIVED:     GREEN,
            AgentState.STUCK:       RED,
            AgentState.INITIALIZING: GRAY,
        }
        color = state_colors.get(state, BLUE)

        # Draw agent body (circle)
        pygame.draw.circle(self.screen, color, (ax, ay), radius)
        pygame.draw.circle(self.screen, WHITE, (ax, ay), radius, 2)

        # Draw direction indicator
        angle_rad = math.radians(agent.angle)
        tip_x = ax + int(math.cos(angle_rad) * (radius - 2))
        tip_y = ay + int(math.sin(angle_rad) * (radius - 2))
        pygame.draw.line(self.screen, WHITE, (ax, ay), (tip_x, tip_y), 3)

        # Inner dot
        pygame.draw.circle(self.screen, WHITE, (ax, ay), 3)

    def _draw_sim_border(self):
        """Draw the border between simulation and dashboard."""
        pygame.draw.line(self.screen, CYAN,
                         (SIM_WIDTH, 0), (SIM_WIDTH, WINDOW_HEIGHT), 2)

    def _draw_dashboard(self, agent, state, status, stats):
        """Draw the right-panel information dashboard."""
        x = SIM_WIDTH + 10
        y = 15
        w = DASHBOARD_WIDTH - 20

        def text(msg, size='medium', color=WHITE, offset_x=0):
            nonlocal y
            font = self.font_large if size == 'large' else \
                   self.font_medium if size == 'medium' else \
                   self.font_small
            surf = font.render(msg, True, color)
            self.screen.blit(surf, (x + offset_x, y))
            y += surf.get_height() + 4

        def divider(color=GRAY):
            nonlocal y
            pygame.draw.line(self.screen, color,
                             (x, y), (x + w, y), 1)
            y += 8

        # ── Title ────────────────────────────────────────────────
        text("AI NAV SYSTEM", 'large', CYAN)
        divider(CYAN)

        # ── Status ───────────────────────────────────────────────
        state_colors = {
            AgentState.MOVING:       GREEN,
            AgentState.REPLANNING:   ORANGE,
            AgentState.WAITING:      YELLOW,
            AgentState.ARRIVED:      GREEN,
            AgentState.STUCK:        RED,
            AgentState.INITIALIZING: LIGHT_GRAY,
        }
        sc = state_colors.get(state, WHITE)
        text(f"State: {state}", 'medium', sc)
        # Wrap long status messages
        if len(status) > 22:
            text(status[:22], 'small', LIGHT_GRAY)
            text(status[22:], 'small', LIGHT_GRAY)
        else:
            text(status, 'small', LIGHT_GRAY)
        divider()

        # ── Navigation Info ──────────────────────────────────────
        text("NAVIGATION", 'medium', CYAN)
        text(f"Distance:  {agent.total_distance/30:.1f} cells", 'small')
        text(f"Steps:     {agent.steps_taken}", 'small')
        text(f"Replans:   {agent.replan_count}", 'small',
             ORANGE if agent.replan_count > 0 else WHITE)
        text(f"Waypoints: {len(agent.get_remaining_waypoints())}", 'small')
        divider()

        # ── Path Planner Info ────────────────────────────────────
        text("PATH PLANNER", 'medium', CYAN)
        text(f"Algorithm: A*", 'small')
        text(f"Diagonal:  {'ON' if stats.get('diagonal') else 'OFF'}", 'small')
        text(f"Nodes exp: {stats.get('nodes_explored', 0)}", 'small')
        text(f"Path len:  {stats.get('path_length', 0)}", 'small')
        divider()

        # ── Sensor Info ──────────────────────────────────────────
        text("LIDAR SENSOR", 'medium', CYAN)
        hits = sum(1 for r in stats.get('scan', []) if r.get('hit'))
        text(f"Rays:  {stats.get('num_rays', 0)}", 'small')
        text(f"Hits:  {hits}", 'small', RED if hits > 5 else WHITE)
        text(f"Range: {stats.get('sensor_range', 0)} cells", 'small')
        divider()

        # ── Performance ──────────────────────────────────────────
        text("PERFORMANCE", 'medium', CYAN)
        text(f"FPS:   {stats.get('fps', 0):.0f}", 'small', GREEN)
        text(f"Frame: {self.frame_count}", 'small')
        divider()

        # ── Legend ───────────────────────────────────────────────
        text("LEGEND", 'medium', CYAN)
        legend_items = [
            (GREEN,   "S  = Start"),
            (RED,     "G  = Goal"),
            (BLUE,    "◉  = Agent"),
            (PATH_COLOR, "── = Path"),
            (ORANGE,  "■  = Dynamic obs"),
            (GRAY,    "█  = Static obs"),
            (CYAN,    "·  = Trail"),
        ]
        for color, label in legend_items:
            pygame.draw.rect(self.screen, color,
                             pygame.Rect(x, y+2, 10, 10))
            surf = self.font_small.render(label, True, LIGHT_GRAY)
            self.screen.blit(surf, (x + 16, y))
            y += 16

        y += 10
        divider()

        # ── Controls ─────────────────────────────────────────────
        text("CONTROLS", 'medium', CYAN)
        text("R  = New scenario", 'small', LIGHT_GRAY)
        text("S  = Screenshot", 'small', LIGHT_GRAY)
        text("ESC= Quit", 'small', LIGHT_GRAY)