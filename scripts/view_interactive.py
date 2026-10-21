#!/usr/bin/env python3
"""View the shared docking scenario, with adjustable wall-clock playback speed."""
import argparse
import sys
import time
from pathlib import Path

import mujoco.viewer

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from mujoco_sim.python.sim_runner import SimulationRunner


def launch_interactive_viewer(playback_speed: float = 1.0, sync_rate_hz: float = 60.0):
    """Run physics once through SimulationRunner and display each due frame."""
    if playback_speed <= 0 or sync_rate_hz <= 0:
        raise ValueError("playback_speed and sync_rate_hz must be positive")
    print("Launching shared MuJoCo docking scenario")
    print("Mouse: orbit, pan, zoom. Close the viewer to stop.")
    runner = SimulationRunner()
    with mujoco.viewer.launch_passive(runner.world.model, runner.world.data) as viewer:
        viewer.cam.lookat = [0.0, 0.0, 0.95]
        viewer.cam.distance = 2.5
        viewer.cam.elevation = -20.0
        viewer.cam.azimuth = 45.0
        wall_start = time.monotonic()
        next_sync = 0.0

        def update_viewer(active_runner, row):
            nonlocal next_sync
            if not viewer.is_running():
                return False
            elapsed = row['time'] / playback_speed
            if elapsed + 1e-10 >= next_sync:
                viewer.sync()
                next_sync = elapsed + 1.0 / sync_rate_hz
            # Accumulated deadlines include render cost and avoid per-step drift.
            remaining = wall_start + elapsed - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

        runner.run_scenario(duration=7.5, dock_cmd_time=1.0,
                            undock_cmd_time=5.0, step_callback=update_viewer)
        if viewer.is_running():
            viewer.sync()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Simulation seconds per wall-clock second (default: 1)')
    parser.add_argument('--sync-rate', type=float, default=60.0,
                        help='Viewer sync frequency in Hz (default: 60)')
    args = parser.parse_args()
    launch_interactive_viewer(args.speed, args.sync_rate)
