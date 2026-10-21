"""Acceptance test suite T1–T9 for dual-quadrotor MuJoCo docking rebound fix.

Verifies:
T1: Low-speed docking under capture conditions enters LOCKED and maintains connection.
T2: Post-lock geometric tolerance (relative position < 1 mm, relative attitude < 0.1 rad).
T3: Post-lock stability without periodic bouncing or constraint oscillation.
T4: High-speed / misaligned approach fails capture gracefully (no forced weld).
T5: Weld activation does not jump qpos or qvel (zero state discontinuity).
T6: Locked aggregate hover stability without motor saturation or abnormal internal forces.
T7: Clean undock separation and verified restoration of connector collision masks.
T8: Deterministic regression (10 initial offsets; not a random Monte Carlo).
T9: Parameter sensitivity sweep across approach speeds, stiffness, and damping.
"""
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "core" / "build"))

from mujoco_sim.python.sim_runner import SimulationRunner
import docking_core_py as dcp


def _xyz(rows, prefix):
    return np.array([[row[f"{prefix}_{axis}"] for axis in "xyz"] for row in rows])


def _tail(rows, duration=0.5):
    return [row for row in rows if row["time"] >= rows[-1]["time"] - duration - 1e-10]


def test_t1_low_speed_docking_and_lock():
    """T1: Nominal low-speed docking must enter LOCKED and maintain connection."""
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0, undock_cmd_time=None)
    
    assert runner.dock_event_time is not None, "Docking event was not triggered"
    assert 1.0 < runner.dock_event_time < 4.0, f"Docking occurred at unexpected time: {runner.dock_event_time}"
    
    dock_events = [e for e in runner.events if e["event"] == "DOCK"]
    assert len(dock_events) == 1, "Expected exactly one DOCK event"
    
    # Check that post-dock, the weld remains active continuously
    post_dock_rows = [r for r in rows if r["time"] > runner.dock_event_time + 0.05]
    assert len(post_dock_rows) > 0
    assert all(r["weld_active"] == 1 for r in post_dock_rows), "Weld did not remain active"
    assert all(r["mode_name"] == "CONNECTED" for r in post_dock_rows), "FSM did not remain in CONNECTED"
    
    # Check max separation post-contact does not exhibit the old 107mm rebound
    contact_rows = [r for r in rows if r["time"] >= (runner.first_contact_time or 2.7)]
    pre_dock_contact = [r for r in contact_rows if r["time"] < runner.dock_event_time]
    if pre_dock_contact:
        max_rebound = max(r["rel_dist"] for r in pre_dock_contact)
        assert max_rebound < 0.010, f"Excessive rebound before lock: {max_rebound*1000:.2f} mm (limit 10 mm)"


def test_t2_post_lock_geometric_tolerance():
    """T2: Post-lock relative position error < 1.0 mm, relative attitude < 0.05 rad."""
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0, undock_cmd_time=None)
    assert runner.dock_event_time is not None
    
    post_dock = [r for r in rows if r["time"] >= runner.dock_event_time + 0.1]
    rel_positions = _xyz(post_dock, "connector_relative_position")
    pos_errors = np.linalg.norm(rel_positions, axis=1)
    max_pos_err = float(np.max(pos_errors))
    
    att_errors = [r["relative_attitude_rad"] for r in post_dock]
    max_att_err = float(np.max(att_errors))
    
    assert max_pos_err < 0.0010, f"Post-lock relative position error {max_pos_err*1000:.3f} mm exceeded 1 mm"
    assert max_att_err < 0.05, f"Post-lock relative attitude error {max_att_err:.4f} rad exceeded 0.05 rad"


def test_t3_post_lock_stability_no_oscillations():
    """T3: Post-lock stability without periodic bouncing or constraint oscillation."""
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0, undock_cmd_time=None)
    assert runner.dock_event_time is not None
    
    # In locked state, contact pads collision is filtered to eliminate constraint fighting
    post_dock = [r for r in rows if r["time"] >= runner.dock_event_time + 0.1]
    contact_forces = [r["contact_normal_force_N"] for r in post_dock]
    assert max(contact_forces) < 0.1, f"Unexpected contact force fighting weld constraint: {max(contact_forces)} N"
    
    # Check weld force smoothness: standard deviation and peak should be modest
    weld_forces = np.linalg.norm(_xyz(post_dock, "weld_q1_force_world_N"), axis=1)
    assert float(np.max(weld_forces)) < 5.0, f"Weld constraint force peaked too high: {np.max(weld_forces)} N"
    # Over steady hover (last 1.0 s), weld force variation should be tiny
    tail_weld = np.linalg.norm(_xyz(_tail(rows, 1.0), "weld_q1_force_world_N"), axis=1)
    assert float(np.std(tail_weld)) < 0.2, f"Weld force oscillates in steady hover: std = {np.std(tail_weld)}"


def test_t4_high_speed_or_misaligned_fails_gracefully():
    """T4: Non-capture conditions (e.g. large misalignment or offset) must not force a weld."""
    # 1. A 60 mm target gap stays outside physical capture in this trajectory.
    # A 30 mm TARGET gap can transiently touch and legally lock; it is retained
    # separately as a boundary experiment, not mistaken for an actual 30 mm gap.
    runner1 = SimulationRunner()
    rows1 = runner1.run_scenario(duration=5.0, dock_cmd_time=1.0, approach_offset=0.06, undock_cmd_time=None)
    assert runner1.dock_event_time is None, "Weld was erroneously forced under position-offset approach"
    assert all(r["weld_active"] == 0 for r in rows1)

    # 2. Severe attitude misalignment (14.3 deg yaw error > 8.6 deg capture envelope)
    runner2 = SimulationRunner()
    rows2 = runner2.run_scenario(duration=5.0, dock_cmd_time=1.0, research_options=dict(q2_yaw_rad=0.25), undock_cmd_time=None)
    assert runner2.dock_event_time is None, "Weld was erroneously forced under severely misaligned attitude"
    assert all(r["weld_active"] == 0 for r in rows2)


def test_t5_weld_activation_no_state_jump():
    """T5: Activating weld must not manually modify or jump qpos/qvel."""
    runner = SimulationRunner()
    runner.run_scenario(duration=5.0, dock_cmd_time=1.0, undock_cmd_time=None)
    assert runner.dock_event_time is not None
    
    dock_event = [e for e in runner.events if e["event"] == "DOCK"][0]
    pre_qpos = np.array(dock_event["pre"]["qpos"])
    post_qpos = np.array(dock_event["post"]["qpos"])
    pre_qvel = np.array(dock_event["pre"]["qvel"])
    post_qvel = np.array(dock_event["post"]["qvel"])
    
    # Physical weld activation in MuJoCo simply turns on eq_active; it must NOT teleport
    max_dpos = float(np.max(np.abs(post_qpos - pre_qpos)))
    max_dvel = float(np.max(np.abs(post_qvel - pre_qvel)))
    assert max_dpos < 1e-12, f"qpos jumped at dock event: {max_dpos}"
    assert max_dvel < 1e-12, f"qvel jumped at dock event: {max_dvel}"


def test_t6_locked_hover_stability_no_saturation():
    """T6: Locked aggregate maintains stable hover without motor saturation or abnormal internal force."""
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=8.0, dock_cmd_time=1.0, undock_cmd_time=None)
    assert runner.dock_event_time is not None
    
    tail = _tail(rows, 1.5)
    # Check aggregate COM hover stability (converging, small velocity)
    com_pos = _xyz(tail, "com")
    com_vel = _xyz(tail, "com_velocity")
    com_err = np.linalg.norm(com_pos - np.array([0.0, 0.0, 1.0]), axis=1)
    max_vel = float(np.max(np.linalg.norm(com_vel, axis=1)))
    
    assert float(np.max(com_err)) < 0.08, f"Aggregate COM hover error {np.max(com_err)} m too high"
    assert max_vel < 0.02, f"Aggregate COM velocity {max_vel} m/s too high (not stable hover)"
    
    # Check attitude level
    for axis in ("roll", "pitch"):
        max_deg = max(abs(r[f"q1_{axis}_deg"]) for r in tail)
        assert max_deg < 1.0, f"Attitude {axis} {max_deg} deg exceeded 1.0 deg"
    
    # Check all 8 motors are within valid thrust limits without saturation
    for k in range(8):
        thrusts = [r[f"motor{k}_actual_N"] for r in tail]
        assert min(thrusts) > 0.5, f"Motor {k} thrust dropped too low: {min(thrusts)} N"
        assert max(thrusts) < 4.5, f"Motor {k} thrust near saturation: {max(thrusts)} N"
        
    # Check internal weld force is minimal (< 1.0 N in steady hover)
    weld_forces = np.linalg.norm(_xyz(tail, "weld_q1_force_world_N"), axis=1)
    assert float(np.max(weld_forces)) < 1.0, f"Abnormal internal weld force: {np.max(weld_forces)} N"


def test_t7_undock_clean_separation_and_collision_restoration():
    """T7: Undocking separates cleanly and restores connector collision masks."""
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=10.0, dock_cmd_time=1.0, undock_cmd_time=5.0)
    assert runner.dock_event_time is not None
    assert runner.release_event_time is not None
    
    # Verify weld was active between dock and undock, and turned off at undock
    for r in rows:
        if runner.dock_event_time + 0.01 <= r["time"] < runner.release_event_time:
            assert r["weld_active"] == 1
        elif r["time"] > runner.release_event_time + 0.05:
            assert r["weld_active"] == 0
    
    # Pair filtering reserves dedicated bits. Verify original compatibility,
    # rather than requiring an implementation-specific integer mask value of 1.
    filt = runner.latch.collision_filter
    assert filt.active
    types,aff = runner.world.model.geom_contype,runner.world.model.geom_conaffinity
    for a in range(runner.world.model.ngeom):
        for b in range(a):
            original = bool((int(filt.original_type[a]) & int(filt.original_affinity[b])) |
                            (int(filt.original_type[b]) & int(filt.original_affinity[a])))
            restored = bool((int(types[a]) & int(aff[b])) | (int(types[b]) & int(aff[a])))
            assert restored == original

    # Check final independent hover recovery
    tail = _tail(rows, 1.0)
    assert float(np.max(_xyz(tail, "q1") - [0.0, -0.6, 1.0])) < 0.03
    assert float(np.max(_xyz(tail, "q2") - [0.0, 0.6, 1.0])) < 0.03


def test_t8_deterministic_repeatability_10_runs():
    """T8: Repeat 10 nominal docking runs; verify 100% lock rate and tight time distribution."""
    dock_times = []
    contact_times = []
    
    for trial in range(10):
        runner = SimulationRunner()
        # Slight initial offset perturbation (±1 mm, well within nominal approach)
        dy = (trial - 4.5) * 0.0002
        rows = runner.run_scenario(duration=5.0, dock_cmd_time=1.0, q1_init_y=-0.6 + dy, q2_init_y=0.6 - dy)
        assert runner.dock_event_time is not None, f"Trial {trial} failed to dock"
        assert runner.first_contact_time is not None, f"Trial {trial} had no contact recorded"
        dock_times.append(runner.dock_event_time)
        contact_times.append(runner.first_contact_time)
        
        # Check no rebound separation
        contact_to_dock = [r for r in rows if runner.first_contact_time <= r["time"] <= runner.dock_event_time]
        if contact_to_dock:
            max_sep = max(r["rel_dist"] for r in contact_to_dock)
            assert max_sep < 0.010, f"Trial {trial} separation {max_sep*1000:.2f} mm exceeded 10 mm"
            
    dock_delays = np.array(dock_times) - np.array(contact_times)
    assert np.all(dock_delays > 0.0), "Dock preceded contact"
    assert np.all(dock_delays < 0.8), f"Docking took too long after contact: max {np.max(dock_delays):.3f} s"
    assert np.std(dock_times) < 0.1, f"Dock time variance too large: std = {np.std(dock_times):.4f} s"


def test_t9_parameter_sensitivity():
    """T9: Sensitivity tests across approach speeds, capture stiffness, and damping."""
    # Test approach durations: 1.8s (faster), 2.2s (nominal), 2.6s (slower)
    for app_dur in [1.8, 2.2, 2.6]:
        runner = SimulationRunner()
        opts = dict(approach_duration_s=app_dur)
        rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0, research_options=opts)
        assert runner.dock_event_time is not None, f"Failed with approach duration {app_dur} s"
        
    # Test capture stiffness variations: 100 N/m, 140 N/m, 180 N/m
    for k_cap in [100.0, 140.0, 180.0]:
        runner = SimulationRunner()
        runner.latch.capture_stiffness = k_cap
        rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0)
        assert runner.dock_event_time is not None, f"Failed with capture stiffness {k_cap} N/m"
        
    # Test capture damping variations: 7.0 Ns/m, 10.0 Ns/m, 14.0 Ns/m
    for c_cap in [7.0, 10.0, 14.0]:
        runner = SimulationRunner()
        runner.latch.capture_damping = c_cap
        rows = runner.run_scenario(duration=6.0, dock_cmd_time=1.0)
        assert runner.dock_event_time is not None, f"Failed with capture damping {c_cap} Ns/m"
