import sys
from pathlib import Path
import pytest
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / 'core' / 'build'))

from mujoco_sim.python.sim_runner import SimulationRunner
import docking_core_py as dcp


def test_docking_and_undocking_cycle():
    """Verify complete FSM mode progression and weld engagement/disengagement."""
    runner = SimulationRunner()
    logs = runner.run_scenario(duration=7.5, dock_cmd_time=1.0, undock_cmd_time=5.0)

    modes = set(l['mode_name'] for l in logs)
    # Must visit all key hybrid modes
    required_modes = {'FREE', 'APPROACH', 'ALIGN', 'CAPTURE_PENDING', 'CONNECTED', 'RELEASE_PREP', 'SEPARATING', 'RECOVERY'}
    assert required_modes.issubset(modes), f"Missing modes: {required_modes - modes}"

    # Verify weld was active during connected mode
    weld_steps = [l for l in logs if l['weld_active']]
    assert len(weld_steps) > 300, "Weld constraint should be active for multiple steps"

    # Verify quads maintained stable altitude during connected flight
    conn_logs = [l for l in logs if l['mode_name'] == 'CONNECTED']
    avg_z = np.mean([l['q1_z'] for l in conn_logs])
    assert 0.90 < avg_z < 1.10, f"Unstable altitude during connected flight: {avg_z}"


def test_zero_preload_release_velocity_continuity():
    """Verify that releasing with zero preload causes no artificial discontinuous velocity impulse."""
    runner = SimulationRunner()
    logs = runner.run_scenario(duration=6.5, dock_cmd_time=1.0, undock_cmd_time=4.0)

    # Find the transition from RELEASE_PREP to SEPARATING
    sep_idx = None
    for i, l in enumerate(logs):
        if l['mode_name'] == 'SEPARATING':
            sep_idx = i
            break

    assert sep_idx is not None, "SEPARATING mode was never reached"

    # Check pitch rate before and right after weld release
    pre_release = logs[sep_idx - 1]
    post_release = logs[sep_idx]

    delta_pitch_rate = abs(post_release['q1_pitch_rate'] - pre_release['q1_pitch_rate'])
    # Velocity change should be tiny and continuous (no solver explosion)
    assert delta_pitch_rate < 0.5, f"Unphysical angular velocity jump on release: {delta_pitch_rate} rad/s"
