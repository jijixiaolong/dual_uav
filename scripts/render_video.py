#!/usr/bin/env python3
"""Render the same SimulationRunner scenario used by tests and diagnostics."""
import argparse
import subprocess
import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from mujoco_sim.python.sim_runner import SimulationRunner


def render_animation(output_mp4: Path, output_gif: Path, width: int = 1280,
                     height: int = 720, fps: int = 30):
    """Encode frames sampled by simulation time from the shared physics loop."""
    output_mp4, output_gif = Path(output_mp4), Path(output_gif)
    if width <= 0 or height <= 0 or width % 2 or height % 2 or fps <= 0:
        raise ValueError("MP4 dimensions must be positive and even; fps must be positive")
    output_mp4.parent.mkdir(parents=True, exist_ok=True)
    output_gif.parent.mkdir(parents=True, exist_ok=True)
    runner = SimulationRunner()
    renderer = mujoco.Renderer(runner.world.model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat = [0.0, 0.0, 0.95]
    camera.distance = 2.5
    camera.elevation = -20.0
    camera.azimuth = 45.0

    duration = 12.0
    frame_count = 0
    scene_saved = False
    expected_frames = round(duration * fps)
    mode_colors = {
        'FREE': (80, 150, 240), 'APPROACH': (240, 200, 50),
        'ALIGN': (240, 160, 40), 'CAPTURE_PENDING': (255, 100, 100),
        'CAPTURE': (255, 100, 100), 'CONNECTED': (50, 220, 100),
        'RELEASE_PREP': (240, 120, 200), 'PRE_RELEASE': (240, 120, 200),
        'SEPARATING': (200, 100, 240), 'RELEASE': (200, 100, 240),
        'RECOVERY': (100, 200, 240), 'SEPARATED': (100, 200, 240),
    }
    command = [
        'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
        '-vcodec', 'rawvideo', '-s', f'{width}x{height}', '-pix_fmt', 'rgb24',
        '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
        '-crf', '18', '-preset', 'fast', str(output_mp4),
    ]
    print(f"Rendering shared scenario: {duration}s, {width}x{height}, {fps} fps")
    process = None
    try:
        process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

        def capture_frame(active_runner, row):
            nonlocal frame_count, scene_saved
            # Fractional frame deadlines avoid drift from rounding steps-per-frame.
            if (frame_count >= expected_frames or
                    row['time'] + 1e-10 < frame_count / fps):
                return
            renderer.update_scene(active_runner.world.data, camera=camera)
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            mode_name = row['mode_name']
            latched = active_runner.world.is_weld_active()
            s1 = active_runner.world.get_quad_state(1)
            s2 = active_runner.world.get_quad_state(2)
            draw.rectangle([(20, 20), (400, 180)], fill=(20, 24, 32),
                           outline=(80, 90, 110), width=2)
            draw.text((35, 30), f"MODE: [{mode_name}]",
                      fill=mode_colors.get(mode_name, (200, 200, 200)))
            draw.text((35, 58), f"Time: {row['time']:5.2f} s / {duration:.1f} s",
                      fill=(240, 240, 240))
            draw.text((35, 84), f"LATCH: {'LOCKED' if latched else 'OPEN'}",
                      fill=(50, 240, 100) if latched else (180, 180, 180))
            draw.text((35, 110), f"Connector error: {row['rel_dist'] * 1000:6.1f} mm",
                      fill=(220, 220, 220))
            draw.text((35, 136),
                      f"Q1 Pos: [{s1.pos.x:.2f}, {s1.pos.y:.2f}, {s1.pos.z:.2f}]",
                      fill=(100, 180, 255))
            draw.text((35, 154),
                      f"Q2 Pos: [{s2.pos.x:.2f}, {s2.pos.y:.2f}, {s2.pos.z:.2f}]",
                      fill=(255, 120, 100))
            process.stdin.write(image.tobytes())
            if (not scene_saved and latched and active_runner.dock_event_time is not None
                    and row['time'] >= active_runner.dock_event_time+.5):
                image.save(output_mp4.parent/'scene_connected.png')
                scene_saved = True
            frame_count += 1

        runner.run_scenario(duration=duration, dock_cmd_time=1.0,
                            undock_cmd_time=6.0, step_callback=capture_frame)
        process.stdin.close()
        return_code = process.wait()
        error = process.stderr.read().decode(errors='replace')
        if return_code:
            raise RuntimeError(f"FFmpeg MP4 encoding failed ({return_code}): {error}")
    finally:
        renderer.close()
        if process is not None:
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            if process.poll() is None:
                process.terminate()
                process.wait()
            if process.stderr:
                process.stderr.close()

    print(f"Rendered {frame_count} frames -> {output_mp4}")
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(output_mp4),
        '-vf', 'fps=15,scale=640:-1:flags=lanczos,split[s0][s1];'
               '[s0]palettegen[p];[s1][p]paletteuse', str(output_gif),
    ], check=True, stdout=subprocess.DEVNULL)
    print(f"GIF created: {output_gif}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=PROJECT_ROOT / 'results' / 'videos')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()
    render_animation(args.output / 'docking_animation.mp4',
                     args.output / 'docking_animation.gif', args.width, args.height, args.fps)
