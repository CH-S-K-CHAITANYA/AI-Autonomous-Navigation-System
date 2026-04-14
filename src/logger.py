# src/logger.py
# Saves screenshots, video frames, and CSV performance logs.
# All outputs go to the outputs/ directory.

import os
import csv
import time
import pygame
import numpy as np

try:
    import imageio
    IMAGEIO_AVAILABLE = True
except ImportError:
    IMAGEIO_AVAILABLE = False
    print("[Logger] imageio not available — video saving disabled")

from config import (
    SAVE_SCREENSHOTS, SAVE_VIDEO, SAVE_LOGS,
    SCREENSHOT_DIR, VIDEO_PATH, LOG_PATH, VIDEO_FPS
)


class Logger:
    """Handles all output saving for the simulation."""

    def __init__(self):
        # Create output directories
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        os.makedirs(os.path.dirname(VIDEO_PATH), exist_ok=True)
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)

        self.video_frames = []
        self.log_rows = []
        self.run_id = int(time.time())
        self.screenshot_count = 0

        # Initialize CSV log
        if SAVE_LOGS:
            self.csv_file = open(LOG_PATH, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'frame', 'agent_px', 'agent_py',
                'state', 'path_length', 'replan_count',
                'total_distance', 'nodes_explored'
            ])

    def log_frame(self, frame_num, agent, state, path_length, nodes_explored):
        """Log one frame of data to CSV."""
        if not SAVE_LOGS:
            return
        self.csv_writer.writerow([
            frame_num, f"{agent.px:.1f}", f"{agent.py:.1f}",
            state, path_length, agent.replan_count,
            f"{agent.total_distance:.2f}", nodes_explored
        ])

    def capture_video_frame(self, screen):
        """Capture current screen as a video frame."""
        if not SAVE_VIDEO or not IMAGEIO_AVAILABLE:
            return
        # Convert Pygame surface to numpy array
        frame = pygame.surfarray.array3d(screen)
        frame = np.transpose(frame, (1, 0, 2))  # (width,height,3) → (height,width,3)
        self.video_frames.append(frame)

    def save_screenshot(self, screen, label="screenshot"):
        """Save a screenshot with a descriptive name."""
        if not SAVE_SCREENSHOTS:
            return None
        self.screenshot_count += 1
        filename = f"{SCREENSHOT_DIR}/{self.run_id}_{label}_{self.screenshot_count:03d}.png"
        pygame.image.save(screen, filename)
        print(f"[Logger] Screenshot saved: {filename}")
        return filename

    def save_video(self):
        """Save all captured frames as an MP4 video."""
        if not SAVE_VIDEO or not IMAGEIO_AVAILABLE or not self.video_frames:
            return
        print(f"[Logger] Saving video ({len(self.video_frames)} frames)...")
        try:
            imageio.mimwrite(VIDEO_PATH, self.video_frames, fps=VIDEO_FPS)
            print(f"[Logger] Video saved: {VIDEO_PATH}")
        except Exception as e:
            print(f"[Logger] Video save failed: {e}")

    def close(self):
        """Finalize all outputs."""
        self.save_video()
        if SAVE_LOGS:
            self.csv_file.close()
            print(f"[Logger] Log saved: {LOG_PATH}")
        print(f"[Logger] Run complete. Screenshots: {self.screenshot_count}")