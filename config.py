# config.py
# Central configuration file for the AI Autonomous Navigation System
# Changing values here affects the entire simulation

# ──────────────── WINDOW SETTINGS ────────────────
WINDOW_WIDTH = 1200        # Total window width (simulation + dashboard)
WINDOW_HEIGHT = 700        # Window height
SIM_WIDTH = 900            # Simulation area width (left panel)
DASHBOARD_WIDTH = 300      # Dashboard width (right panel)
FPS = 30                   # Frames per second

# ──────────────── GRID SETTINGS ────────────────
CELL_SIZE = 30             # Each grid cell is 30x30 pixels
GRID_COLS = SIM_WIDTH // CELL_SIZE     # Number of columns
GRID_ROWS = WINDOW_HEIGHT // CELL_SIZE # Number of rows

# ──────────────── COLORS ────────────────
BLACK        = (0,   0,   0)
WHITE        = (255, 255, 255)
GRAY         = (40,  40,  40)
LIGHT_GRAY   = (180, 180, 180)
DARK_GRAY    = (25,  25,  25)
RED          = (220, 50,  50)
GREEN        = (50,  200, 50)
BLUE         = (50,  100, 220)
CYAN         = (0,   200, 200)
YELLOW       = (255, 220, 0)
ORANGE       = (255, 140, 0)
PURPLE       = (160, 60,  220)
DARK_GREEN   = (20,  80,  20)
PATH_COLOR   = (0,   150, 255)
SENSOR_COLOR = (255, 255, 0, 80)  # Semi-transparent yellow

# ──────────────── AGENT SETTINGS ────────────────
AGENT_SPEED        = 2.5   # Pixels per frame
AGENT_RADIUS       = 12    # Visual radius
AGENT_SENSOR_RANGE = 5     # LiDAR range in grid cells
NUM_SENSOR_RAYS    = 36    # Number of LiDAR rays (360 / 36 = 10° each)

# ──────────────── OBSTACLE SETTINGS ────────────────
NUM_STATIC_OBSTACLES  = 25  # Static wall/block obstacles
NUM_DYNAMIC_OBSTACLES = 4   # Moving obstacles
DYNAMIC_OBSTACLE_SPEED = 0.8

# ──────────────── PATH PLANNER ────────────────
ALLOW_DIAGONAL = True       # A* allows diagonal movement
REPLAN_COOLDOWN = 45        # Frames between replanning events

# ──────────────── OUTPUT SETTINGS ────────────────
SAVE_SCREENSHOTS = True
SAVE_VIDEO       = True
SAVE_LOGS        = True
OUTPUT_DIR       = "outputs"
SCREENSHOT_DIR   = "outputs/screenshots"
VIDEO_PATH       = "outputs/videos/simulation.mp4"
LOG_PATH         = "outputs/logs/run_log.csv"
VIDEO_FPS        = 30