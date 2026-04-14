# main.py
# Entry point for the AI Autonomous Navigation System simulation.
# Orchestrates all modules: environment, perception, planning, control, rendering.
#
# Run with: python main.py

import sys
import time
import pygame

from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, SIM_WIDTH, FPS,
    ALLOW_DIAGONAL, AGENT_SENSOR_RANGE, NUM_SENSOR_RAYS
)
from simulation.environment import Environment
from src.agent import Agent
from src.perception import LiDARSensor
from src.occupancy_grid import OccupancyGrid
from src.path_planner import PathPlanner
from src.decision_engine import DecisionEngine, AgentState
from src.visualizer import Visualizer
from src.logger import Logger


def run_simulation(seed=42):
    """
    Main simulation loop.
    seed: random seed for reproducible scenarios (press R to change)
    """
    # ── Initialize Pygame ────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("AI Autonomous Navigation System | Python + A*")
    clock = pygame.time.Clock()

    # ── Initialize all modules ───────────────────────────────────
    env          = Environment(seed=seed)
    agent        = Agent(env.start[0], env.start[1])
    sensor       = LiDARSensor(env)
    occ_grid     = OccupancyGrid(env)
    planner      = PathPlanner()
    decision     = DecisionEngine(env, planner)
    visualizer   = Visualizer(screen)
    logger       = Logger()

    # ── State variables ──────────────────────────────────────────
    current_path  = []
    sensor_scan   = []
    frame_num     = 0
    video_frame_interval = 2   # Capture every 2nd frame for video
    auto_screenshot_done = {'initial': False, 'first_replan': False, 'arrived': False}
    fps_tracker   = []

    print("\n" + "="*55)
    print("  AI Autonomous Navigation System — Simulation Started")
    print("="*55)
    print(f"  Start:  {env.start}")
    print(f"  Goal:   {env.goal}")
    print(f"  Static obstacles:  {len(env.static_obstacles)}")
    print(f"  Dynamic obstacles: {len(env.dynamic_obstacles)}")
    print(f"  Grid:   {len(range(0,1))} ")
    print("="*55)
    print("  Controls: R=Reset  S=Screenshot  ESC=Quit")
    print("="*55 + "\n")

    running = True

    while running:
        frame_start = time.time()
        frame_num += 1

        # ── Handle Events ────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    # Reset with new random scenario
                    logger.close()
                    new_seed = seed + frame_num
                    print(f"\n[Main] Resetting simulation (seed={new_seed})")
                    return run_simulation(seed=new_seed)

                elif event.key == pygame.K_s:
                    # Manual screenshot
                    logger.save_screenshot(screen, "manual")

        # ── Update World ─────────────────────────────────────────
        env.update()   # Move dynamic obstacles

        # ── Perception ───────────────────────────────────────────
        sensor_scan = sensor.scan(agent.px, agent.py)
        detected_static = sensor.get_obstacle_cells_in_range(*agent.get_grid_pos())
        dynamic_cells = env.get_dynamic_obstacle_cells()
        occ_grid.update(detected_static, dynamic_cells)

        # ── Decision Engine ──────────────────────────────────────
        new_state, should_replan = decision.update(agent, occ_grid, env.goal)

        # ── Path Planning ────────────────────────────────────────
        if should_replan:
            planning_grid = occ_grid.get_grid_for_planning()
            agent_pos = agent.get_grid_pos()
            is_replan = (new_state == AgentState.REPLANNING)

            path = planner.find_path(agent_pos, env.goal, planning_grid)

            if path:
                path = planner.smooth_path(path)
                agent.set_path(path, is_replan=is_replan)
                current_path = path
                print(f"[Planner] Path found: {len(path)} waypoints | "
                      f"Nodes explored: {planner.nodes_explored}")
            else:
                print(f"[Planner] No path found from {agent_pos} to {env.goal}")

            decision.on_path_computed(path, agent)

        # ── Agent Movement ───────────────────────────────────────
        agent.update()

        # ── Auto-screenshots at key moments ─────────────────────
        if frame_num == 60 and not auto_screenshot_done['initial']:
            logger.save_screenshot(screen, "initial_state")
            auto_screenshot_done['initial'] = True

        if agent.replan_count == 1 and not auto_screenshot_done['first_replan']:
            logger.save_screenshot(screen, "first_replan")
            auto_screenshot_done['first_replan'] = True

        if decision.state == AgentState.ARRIVED and not auto_screenshot_done['arrived']:
            logger.save_screenshot(screen, "goal_reached")
            auto_screenshot_done['arrived'] = True
            print(f"\n[Main] GOAL REACHED!")
            print(f"       Total distance:  {agent.total_distance/30:.1f} cells")
            print(f"       Steps taken:     {agent.steps_taken}")
            print(f"       Replanning events: {agent.replan_count}")

        # ── FPS Calculation ──────────────────────────────────────
        frame_time = time.time() - frame_start
        fps_tracker.append(1.0 / max(frame_time, 0.001))
        if len(fps_tracker) > 30:
            fps_tracker.pop(0)
        avg_fps = sum(fps_tracker) / len(fps_tracker)

        # ── Stats package for dashboard ──────────────────────────
        stats = {
            'fps':            avg_fps,
            'diagonal':       ALLOW_DIAGONAL,
            'nodes_explored': planner.nodes_explored,
            'path_length':    len(current_path),
            'scan':           sensor_scan,
            'num_rays':       NUM_SENSOR_RAYS,
            'sensor_range':   AGENT_SENSOR_RANGE,
        }

        # ── Render ───────────────────────────────────────────────
        visualizer.render(
            env, agent, occ_grid, current_path,
            sensor_scan, decision.state, decision.status_message, stats
        )
        pygame.display.flip()

        # ── Logging ──────────────────────────────────────────────
        if frame_num % video_frame_interval == 0:
            logger.capture_video_frame(screen)

        if frame_num % 30 == 0:
            logger.log_frame(
                frame_num, agent, decision.state,
                len(current_path), planner.nodes_explored
            )

        # ── Timing ───────────────────────────────────────────────
        clock.tick(FPS)

    # ── Cleanup ──────────────────────────────────────────────────
    logger.close()
    pygame.quit()
    print("\n[Main] Simulation ended.")
    sys.exit(0)


if __name__ == "__main__":
    run_simulation(seed=42)