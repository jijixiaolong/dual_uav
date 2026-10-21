"""P0/P1 closed-loop acceptance tests using the real MuJoCo motor chain.

The tolerances are engineering acceptance limits, not ECLR performance claims.
All trajectories run with compensation disabled; connected-mode position error
is measured at the aggregate COM, since independent references are inactive.
"""
from pathlib import Path
import sys

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "core" / "build"))

from mujoco_sim.python.sim_runner import SimulationRunner


def _xyz(rows, prefix):
    return np.array([[row[f"{prefix}_{axis}"] for axis in "xyz"] for row in rows])


def _tail(rows, duration=0.5):
    return [row for row in rows if row["time"] >= rows[-1]["time"] - duration - 1e-10]


def _max_norm(values):
    return float(np.max(np.linalg.norm(values, axis=1)))


def _assert_schedule_and_inputs(runner, rows):
    times = np.array([row["time"] for row in rows])
    assert np.allclose(np.diff(times), runner.dt_sim, rtol=0, atol=1e-10)
    ticks = [row for row in rows if row["control_updated"]]
    expected = round(rows[-1]["time"] / runner.control_dt)
    assert len(ticks) == expected
    intervals = np.array([row["control_dt_s"] for row in ticks])
    assert np.allclose(intervals[1:], np.diff([row["time"] for row in ticks]), atol=1e-10)
    # 5 ms control periods on 2 ms physics ticks must alternate between 4/6 ms.
    assert set(np.round(intervals[1:], 6)) == {0.004, 0.006}
    assert abs(intervals[1:].mean() - runner.control_dt) < 3e-6
    minimum = np.r_[[runner.q1_params.min_thrust] * 4, [runner.q2_params.min_thrust] * 4]
    maximum = np.r_[[runner.q1_params.max_thrust] * 4, [runner.q2_params.max_thrust] * 4]
    rate = np.r_[[runner.q1_params.slew_rate_limit] * 4, [runner.q2_params.slew_rate_limit] * 4]
    for kind in ("command", "target", "actual"):
        motors = np.array([[row[f"motor{i}_{kind}_N"] for i in range(8)] for row in rows])
        assert np.isfinite(motors).all()
        assert np.all(motors >= minimum - 1e-12)
        assert np.all(motors <= maximum + 1e-12)
        if kind != "command":
            assert np.all(np.abs(np.diff(motors, axis=0)) <= rate * runner.dt_sim + 1e-10)
    command_ticks = np.array([[row[f"motor{i}_command_N"] for i in range(8)] for row in ticks])
    assert np.all(np.abs(np.diff(command_ticks, axis=0)) <= intervals[1:, None] * rate + 1e-10)
    assert all(row["u_comp_active"] == 0 for row in rows)
    assert all(row[f"motor{i}_compensation_N"] == 0 for row in rows for i in range(8))


def _assert_level(tail, tolerance_deg):
    assert max(abs(row[f"q{i}_{axis}_deg"]) for row in tail for i in (1, 2) for axis in ("roll", "pitch")) < tolerance_deg


def test_independent_closed_loop_hover():
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=3.0, dock_cmd_time=100.0, undock_cmd_time=None)
    tail = _tail(rows)
    assert all(row["weld_active"] == 0 and row["mode_name"] == "FREE" for row in rows)
    assert runner.events == []
    for i, y in ((1, -0.6), (2, 0.6)):
        assert _max_norm(_xyz(tail, f"q{i}") - [0.0, y, 1.0]) < 1e-8
        assert _max_norm(_xyz(tail, f"q{i}_velocity")) < 1e-8
    _assert_level(tail, 1e-6)
    _assert_schedule_and_inputs(runner, rows)


def test_preconnected_aggregate_hover():
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=10.0, preconnected=True, undock_cmd_time=None)
    tail = _tail(rows)
    assert all(row["weld_active"] == 1 and row["mode_name"] == "CONNECTED" for row in rows)
    assert _max_norm(_xyz(tail, "com") - [0.0, 0.0, 1.0]) < 0.01
    assert _max_norm(_xyz(tail, "com_velocity")) < 0.02
    assert max(row["rel_dist"] for row in tail) < 0.002
    _assert_level(tail, 0.2)
    # This fixture starts at rest, level, with balanced hover thrust and zero
    # preload. Large opposing contact/weld forces would be solver artifacts,
    # even if the COM stayed inside its tracking tolerance.
    assert max(row["contact_normal_force_N"] for row in rows) < 0.1
    assert _max_norm(_xyz(rows, "weld_q1_force_world_N")) < 0.1
    _assert_schedule_and_inputs(runner, rows)


def test_preconnected_aggregate_tracks_smooth_reference():
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=7.0, preconnected=True, connected_trajectory=True, undock_cmd_time=None)
    tail = _tail(rows)
    assert all(row["weld_active"] == 1 for row in rows)
    assert _max_norm(_xyz(tail, "com") - [0.12, 0.06, 1.05]) < 0.01
    assert _max_norm(_xyz(tail, "com_velocity")) < 0.01
    assert max(row["aggregate_pos_error_m"] for row in tail) < 0.01
    _assert_level(tail, 1.0)
    _assert_schedule_and_inputs(runner, rows)


def test_capture_release_and_independent_hover_recovery():
    runner = SimulationRunner()
    rows = runner.run_scenario(duration=12.0, dock_cmd_time=1.0, undock_cmd_time=6.0)
    assert runner.dock_event_time is not None and runner.release_event_time is not None
    assert 1.0 < runner.dock_event_time < 6.0 < runner.release_event_time < 6.3
    assert runner.release_event_time - runner.dock_event_time > 0.8
    assert len([event for event in runner.events if event["event"] == "DOCK"]) == 1
    assert len([event for event in runner.events if event["event"] == "UNDOCK"]) == 1
    before_release = min(rows, key=lambda row: abs(row["time"] - (runner.release_event_time - runner.dt_sim)))
    release_window = [row for row in rows if runner.release_event_time <= row["time"] <= runner.release_event_time + 1.2]
    for i, params in ((1, runner.q1_params), (2, runner.q2_params)):
        # A slightly tilted captured pair must not be driven to huge internal
        # moments just to reduce its uncommanded horizontal body force.
        assert np.linalg.norm([before_release[f"q{i}_rotor_torque_body_{axis}"] for axis in "xyz"]) < 0.02
        margin = 0.1 * (params.max_thrust - params.min_thrust)
        assert all(params.min_thrust + margin < before_release[f"motor{k}_actual_N"] < params.max_thrust - margin
                   for k in range(4 * (i - 1), 4 * i))
        # Compare with the actual desired attitude, including the separation
        # trajectory tilt, rather than confusing absolute roll with error.
        assert max(np.rad2deg(row[f"q{i}_attitude_error_rad"]) for row in release_window) < 5.0
        assert _max_norm(_xyz(release_window, f"q{i}_omega_body")) < 0.8
    names = list(dict.fromkeys(row["mode_name"] for row in rows))
    assert names == ["FREE", "APPROACH", "ALIGN", "CAPTURE_PENDING", "CONNECTED", "RELEASE_PREP", "SEPARATING", "RECOVERY"]
    tail = _tail(rows)
    assert all(row["weld_active"] == 0 and row["mode_name"] == "FREE" for row in tail)
    for i, y in ((1, -0.6), (2, 0.6)):
        assert _max_norm(_xyz(tail, f"q{i}") - [0.0, y, 1.0]) < 0.025
        assert _max_norm(_xyz(tail, f"q{i}_velocity")) < 0.01
    _assert_level(tail, 0.2)
    _assert_schedule_and_inputs(runner, rows)


def test_reused_runner_resets_integrators_and_allocator_history():
    runner = SimulationRunner()
    options = dict(duration=3.0, preconnected=True, connected_trajectory=True, undock_cmd_time=None)
    first = runner.run_scenario(**options)
    first_events = runner.events.copy()
    second = runner.run_scenario(**options)
    # Exact replay tests all logged states, forces, motor histories and references.
    assert second == first
    assert runner.events == first_events


def test_aggregate_attitude_is_invariant_to_antipodal_quaternion_encoding():
    runner = SimulationRunner()
    runner.run_scenario(duration=0.02, preconnected=True, undock_cmd_time=None)
    original = runner.aggregate_state()
    expected_rotation = runner.world.rotation(1)
    expected_omega = np.array([original.omega.x, original.omega.y, original.omega.z])
    for quad in (2, 1):
        adr = getattr(runner.world, f"q{quad}_qpos_adr")
        runner.world.data.qpos[adr + 3:adr + 7] *= -1
        runner.world.forward()
        aggregate = runner.aggregate_state()
        encoded = np.array([getattr(aggregate.quat, axis) for axis in "wxyz"])
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, encoded)
        assert np.isfinite(encoded).all()
        assert abs(np.linalg.norm(encoded) - 1.0) < 1e-12
        assert np.allclose(rotation.reshape(3, 3), expected_rotation, atol=1e-12)
        assert np.allclose([aggregate.omega.x, aggregate.omega.y, aggregate.omega.z], expected_omega, atol=1e-12)
