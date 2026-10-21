"""Boundary, censoring, and physical-time regressions for formal event metrics."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from analysis_tools.metrics import evaluate_events, evaluate_window, event_window, settling_time


def row(t, attitude_deg=0.0, position=0.0, omega_deg_s=0.0, motor=2.0, mode="CONNECTED"):
    r = {"time": t, "mode_name": mode, "weld_active": int(mode == "CONNECTED"),
         "aggregate_controller_active": int(mode == "CONNECTED"),
         "contact_normal_force_N": 0.0, "rel_dist": 0.2,
         "aggregate_desired_thrust_N": 20.0, "control_updated": 1, "control_dt_s": 0.1}
    for axis in "xyz":
        r[f"aggregate_desired_torque_Nm_{axis}"] = 0.0
        r[f"weld_q1_force_world_N_{axis}"] = 0.0
    for q in (1, 2):
        r[f"q{q}_attitude_error_rad"] = np.deg2rad(attitude_deg)
        r[f"q{q}_roll_error_deg"] = attitude_deg
        r[f"q{q}_pitch_error_deg"] = 0.0
        r[f"q{q}_roll_deg"] = 10.0 + attitude_deg
        r[f"q{q}_pitch_deg"] = 0.0
        r[f"q{q}_pos_error_m"] = position
        r[f"q{q}_desired_thrust_N"] = 10.0
        for axis in "xyz":
            r[f"q{q}_omega_body_{axis}"] = np.deg2rad(omega_deg_s) if axis == "x" else 0.0
            r[f"q{q}_desired_torque_Nm_{axis}"] = 0.0
            r[f"q{q}_rotor_force_world_{axis}"] = 10.0 if axis == "z" else 0.0
            r[f"q{q}_rotor_torque_body_{axis}"] = 0.0
    for m in range(8):
        for kind in ("command", "target", "actual"):
            r[f"motor{m}_{kind}_N"] = motor
    return r


def event_result(rows, t0=0.0, event="UNDOCK", **config):
    result = evaluate_events(rows, [{"event": event, "time": t0}],
                             {"window_s": 1.0, "expected_events": [], **config})
    return next(r for r in result["events"] if r["occurred"])


@pytest.mark.parametrize("values,expected,censored,status", [
    ([0.0, 0.1, 0.2], 0.0, False, "within_tolerance_from_start"),
    ([1.0, 1.0, 1.0], 0.0, False, "within_tolerance_from_start"),
    ([2.0, 1.5, 1.1], None, True, "unrecovered"),
    ([0.0, 2.0, 0.0], 1.5, False, "recovered"),
    ([2.0, 0.0, 2.0], None, True, "unrecovered"),
    ([2.0, 1.0, 0.0], 1.0, False, "recovered"),
    ([2.0, 2.0, 1.0], 2.0, False, "recovered"),
    ([0.0, 0.0, 2.0], None, True, "unrecovered"),
])
def test_recovery_boundary_and_reentry_cases(values, expected, censored, status):
    result = settling_time([0.0, 1.0, 2.0], values)
    assert result["time_s"] == expected
    assert result["right_censored"] is censored
    assert result["status"] == status


def test_joint_recovery_uses_last_signal_to_cross_tolerance():
    result = settling_time([5.0, 6.0], [[3.0, 2.0], [0.0, 0.8]])
    assert result["time_s"] == pytest.approx(5/6)


def test_recovery_missing_values_are_unavailable_and_json_safe():
    assert settling_time([0, 1], [np.nan, 0])["status"] == "unavailable"
    rows = [row(0), row(1)]
    del rows[0]["q1_pos_error_m"]
    result = evaluate_events(rows, [{"event": "DOCK", "time": 0}])
    dock = next(r for r in result["events"] if r["event"] == "DOCK")
    assert dock["metrics"]["q1"]["settling_time_s"] is None
    json.dumps(result, allow_nan=False)


def test_zero_recovery_does_not_regress_to_window_length():
    result = event_result([row(0), row(.2), row(1)])
    assert result["metrics"]["combined"]["settling_time_s"] == 0.0
    assert result["metrics"]["combined"]["right_censored"] is False


def test_never_recovered_is_none_not_nominal_window_length():
    result = event_result([row(0, attitude_deg=2), row(1, attitude_deg=2)])
    assert result["metrics"]["combined"]["settling_time_s"] is None
    assert result["metrics"]["combined"]["right_censored"] is True


def test_exact_t0_and_t1_are_interpolated_not_rounded_to_first_mode_sample():
    rows = [row(0.0, attitude_deg=0), row(.5, attitude_deg=2), row(1.5, attitude_deg=0)]
    result = event_result(rows, t0=.25)
    assert result["window"]["observed_start_s"] == .25
    assert result["window"]["observed_end_s"] == 1.25
    assert result["window"]["observed_duration_s"] == 1.0
    # Areas: .25*(1+2)/2 + .75*(2+.5)/2.
    assert result["metrics"]["q1"]["attitude_error_integral_deg_s"] == pytest.approx(1.3125)
    assert result["metrics"]["q1"]["attitude_recovery"]["time_s"] == pytest.approx(1.0)


def test_recording_spanning_modes_is_not_filtered_to_connected():
    rows = [row(0, mode="CONNECTED"), row(.5, mode="SEPARATING", attitude_deg=4),
            row(1, mode="RECOVERY", attitude_deg=3)]
    result = event_result(rows, event="DOCK")
    assert result["window"]["complete"]
    assert result["metrics"]["combined"]["peak_attitude_error_deg"] == 4.0
    assert result["metrics"]["combined"]["settling_time_s"] is None


def test_short_recording_marks_truncation_independently_of_recovery():
    result = event_result([row(0), row(.4)])
    assert not result["window"]["complete"]
    assert result["window"]["right_truncated"]
    assert result["metrics"]["combined"]["settling_time_s"] == 0.0
    assert result["metrics"]["combined"]["right_censored"] is False
    assert result["window"]["observed_duration_s"] == .4


def test_left_truncated_recording_cannot_claim_zero_event_recovery():
    result = event_result([row(.1), row(1)])
    assert result["window"]["left_truncated"]
    assert result["metrics"]["combined"]["settling_time_s"] is None
    assert result["metrics"]["combined"]["joint_recovery"]["status"] == "left_truncated"


def test_missing_event_and_failed_trial_are_retained():
    result = evaluate_events([row(0, mode="APPROACH"), row(2, mode="ALIGN")],
                             [{"event": "CONTACT", "time": .5}])
    assert result["outcome"]["category"] == "timeout"
    assert result["outcome"]["reason"] == "capture_timeout"
    assert len(result["events"]) == 5
    assert next(r for r in result["events"] if r["event"] == "DOCK")["occurred"] is False


@pytest.mark.parametrize("mode,event,reason", [
    ("APPROACH", None, "no_contact"),
    ("ABORT", None, "fsm_abort"),
    ("CONNECTED", "DOCK", "release_not_observed"),
])
def test_failure_classification(mode, event, reason):
    events = [{"event": event, "time": 0}] if event else []
    result = evaluate_events([row(0, mode=mode), row(2, mode=mode)], events)
    assert result["outcome"]["reason"] == reason
    assert result["outcome"]["failed"]


def test_release_event_requires_observed_physical_separation_for_success():
    events = [{"event": "DOCK", "time": 0}, {"event": "UNDOCK", "time": 1}]
    rows = [row(0), row(1, mode="SEPARATING"), row(2, mode="RECOVERY")]
    result = evaluate_events(rows, events)
    assert result["outcome"]["category"] == "success"
    for r in rows:
        r["rel_dist"] = .001
    result = evaluate_events(rows, events)
    assert result["outcome"]["reason"] == "release_without_observed_separation"


def test_all_occurrences_including_repeated_capture_ready_are_kept():
    events = [{"event": "CAPTURE_READY", "time": .1}, {"event": "CAPTURE_READY", "time": .4}]
    result = evaluate_events([row(0), row(2)], events)
    occurrences = [r for r in result["events"] if r["event"] == "CAPTURE_READY"]
    assert [r["occurrence"] for r in occurrences] == [0, 1]


def test_uneven_timestep_integrals_use_physical_time():
    fine = [row(t, attitude_deg=2*t, position=.1*t, motor=t+1) for t in [0, .1, .3, 1]]
    coarse = [row(t, attitude_deg=2*t, position=.1*t, motor=t+1) for t in [0, 1]]
    a, b = event_result(fine)["metrics"], event_result(coarse)["metrics"]
    for metrics in (a, b):
        assert metrics["q1"]["attitude_error_integral_deg_s"] == pytest.approx(1.0)
        assert metrics["q1"]["position_error_integral_m_s"] == pytest.approx(.05)
        assert metrics["q1"]["attitude_error_squared_integral_deg2_s"] == pytest.approx(4/3)
        assert metrics["motors"]["actual"]["thrust_squared_integral_N2_s"] == pytest.approx(8*7/3)


def test_eight_motor_saturation_is_time_weighted_not_sample_average():
    rows = [row(t, motor=2) for t in [0, .1, .9, 1.0]]
    rows[0]["motor0_command_N"] = 5.5
    rows[1]["motor0_command_N"] = 5.5
    rows[2]["motor1_command_N"] = 5.5
    result = event_result(rows)["metrics"]["motors"]["command"]
    # Motor zero saturated for .9s, motor one for .1s -> 1 / 8 motor-seconds.
    assert result["saturation_fraction"] == pytest.approx(1/8)
    assert result["saturation_fraction_per_motor"][:2] == pytest.approx([.9, .1])


def test_motor_command_boundary_interpolation_preserves_zero_order_hold():
    rows = [row(0, motor=1), row(.5, motor=4), row(1.5, motor=4)]
    result = event_result(rows, t0=.25)["metrics"]["motors"]["command"]
    assert result["thrust_squared_integral_N2_s"] == pytest.approx(8*(.25+16*.75))


def test_motor_control_tick_slope_is_not_physics_sample_slope():
    rows = [row(0, motor=1), row(.002, motor=1.1), row(1, motor=1.1)]
    rows[1]["control_dt_s"] = .006
    rows[2]["control_updated"] = 0
    result = event_result(rows)["metrics"]["motors"]["command"]
    assert result["peak_sample_slope_N_s"] == pytest.approx(50)
    assert result["peak_control_update_slope_N_s"] == pytest.approx(100/6)


def test_stale_inactive_wrenches_are_excluded():
    rows = [row(0), row(.5, mode="SEPARATING"), row(1, mode="RECOVERY")]
    rows[0]["q1_desired_thrust_N"] = rows[0]["q2_desired_thrust_N"] = 999
    rows[1]["aggregate_desired_thrust_N"] = rows[2]["aggregate_desired_thrust_N"] = 999
    result = event_result(rows)["metrics"]["control"]
    # .5s aggregate 20^2 + .5s independent (10^2+10^2).
    assert result["active_desired_thrust_squared_integral_N2_s"] == pytest.approx(300)
    assert result["actual_rotor_force_squared_integral_N2_s"] == pytest.approx(200)
    assert result["energy_is_mechanical_work_J"] is False


def test_quaternion_boundary_interpolation_handles_antipodal_encoding():
    rows = [row(0), row(1), row(2)]
    for n, r in enumerate(rows):
        for quad in ("q1", "q2"):
            for prefix in (f"{quad}_quat", f"{quad}_desired_quat"):
                for axis, value in zip("wxyz", [(-1)**n, 0, 0, 0]):
                    r[f"{prefix}_{axis}"] = value
    result = event_result(rows, t0=.3)
    assert result["metrics"]["combined"]["peak_attitude_error_deg"] == 0.0


def test_so3_error_does_not_use_absolute_roll_as_tracking_error():
    result = event_result([row(0, attitude_deg=.1), row(1, attitude_deg=.2)])["metrics"]["q1"]
    assert result["peak_attitude_error_deg"] == pytest.approx(.2)
    assert result["peak_absolute_roll_deg"] == pytest.approx(10.2)


@pytest.mark.parametrize("events,rows", [
    ([], []),
    ([{"event": "DOCK", "time": 4}], [row(0), row(1)]),
    ([{"event": "DOCK", "time": 0}], [row(0)]),
])
def test_missing_telemetry_no_overlap_and_single_sample_do_not_crash(events, rows):
    result = evaluate_events(rows, events)
    json.dumps(result, allow_nan=False)


def test_csv_string_rows_are_accepted():
    rows = [{key: str(value) for key, value in row(t).items()} for t in [0, 1]]
    result = event_result(rows)
    assert result["metrics"]["q1"]["peak_attitude_error_deg"] == 0.0


@pytest.mark.parametrize("times", [[0, 0], [1, 0], [0, np.nan]])
def test_invalid_timestamps_are_rejected(times):
    with pytest.raises(ValueError, match="timestamps"):
        evaluate_events([row(t) for t in times], [])


def test_per_event_windows_are_independent():
    events = [{"event": "CONTACT", "time": .1}, {"event": "DOCK", "time": .2},
              {"event": "UNDOCK", "time": .3}]
    result = evaluate_events([row(0), row(3)], events,
                             {"event_windows_s": {"CONTACT": .4, "DOCK": .8, "UNDOCK": 1.2}})
    for record, duration in zip(result["events"][:3], [.4, .8, 1.2]):
        assert record["window"]["observed_duration_s"] == pytest.approx(duration)


def test_custom_motor_limits_and_actual_linear_saturation_occupancy():
    rows = [row(0, motor=1), row(1, motor=3)]
    result = event_result(rows, motor_min_N=[0]*8, motor_max_N=[2]*8, saturation_epsilon_N=0)
    assert result["metrics"]["motors"]["actual"]["saturation_fraction"] == pytest.approx(.5)


def test_declared_intervention_window_matches_event_metrics_without_fake_events():
    rows = [row(0), row(.5, attitude_deg=2), row(2)]
    declared = evaluate_window(rows, .25, duration_s=1, config={"expected_events": []})
    actual_event = event_result(rows, t0=.25)
    assert declared["window"] == actual_event["window"]
    assert declared["metrics"] == actual_event["metrics"]
    assert "events" not in declared and "outcome" not in declared
    assert all(r["weld_active"] for r in rows)
    json.dumps(declared, allow_nan=False)


def test_declared_intervention_window_preserves_no_observation_case():
    declared = evaluate_window([], 1.0)
    assert declared["metrics"] is None
    assert declared["window"]["status"] == "missing_telemetry"


def test_post_step_logger_digital_inputs_use_right_zoh():
    rows = [row(0, motor=1), row(.4, motor=2), row(1, motor=3)]
    for r in rows:
        r["controller_state_time_s"] = r["time"] - .002
    rows[1]["aggregate_desired_thrust_N"] = 10
    rows[2]["aggregate_desired_thrust_N"] = 20
    result = event_result(rows)["metrics"]
    assert result["motors"]["command"]["thrust_squared_integral_N2_s"] == pytest.approx(8*(.4*4+.6*9))
    assert result["control"]["active_desired_thrust_squared_integral_N2_s"] == pytest.approx(.4*100+.6*400)
    assert result["motors"]["command"]["sample_hold"] == "right_zero_order"


def test_post_step_logger_synthetic_endpoint_uses_interval_input():
    rows = [row(0, motor=1), row(.4, motor=2), row(1, motor=3)]
    for r in rows:
        r["controller_state_time_s"] = r["time"] - .002
    result = evaluate_window(rows, .1, duration_s=.6)["metrics"]["motors"]["command"]
    assert result["thrust_squared_integral_N2_s"] == pytest.approx(8*(.3*4+.3*9))


@pytest.mark.parametrize("quantity,limit,value,config", [
    ("q1_z", .2, .1, "min_altitude_m"),
    ("q1_roll_deg", 60, 70, "max_tilt_deg"),
    ("q1_velocity_x", 5, 6, "max_speed_mps"),
])
def test_completed_events_with_finite_physical_crash_are_failure(quantity, limit, value, config):
    rows = [row(0), row(1, mode="SEPARATING"), row(2, mode="RECOVERY")]
    for r in rows:
        for q in (1, 2):
            r[f"q{q}_z"] = 1.0
            for axis in "xyz":
                r[f"q{q}_velocity_{axis}"] = 0.0
    rows[-1][quantity] = value
    events = [{"event": "DOCK", "time": 0}, {"event": "UNDOCK", "time": 1}]
    result = evaluate_events(rows, events, {config: limit})
    assert result["outcome"]["category"] == "failure"
    assert result["outcome"]["reason"] == "flight_envelope_violation"
    assert result["outcome"]["flight_envelope"]["violation"]
    assert result["outcome"]["docking_success"]  # Event occurrence remains auditable.


@pytest.mark.parametrize("key", ["simulation_duration_s", "expected_end_time_s"])
def test_observation_ending_early_is_classified_even_if_events_completed(key):
    events = [{"event": "DOCK", "time": 0}, {"event": "UNDOCK", "time": .5}]
    result = evaluate_events([row(0), row(1, mode="RECOVERY")], events, {key: 2.0})
    assert result["outcome"]["reason"] == "observation_ended_early"
    assert result["outcome"]["timed_out"]


def test_envelope_checks_are_optional_for_generic_inputs():
    rows = [row(0, mode="APPROACH"), row(1, mode="APPROACH")]
    rows[-1]["q1_z"] = -.1
    result = evaluate_events(rows, [], {"expected_events": []})
    assert result["outcome"]["category"] == "success"


class LegacyFakeRunner:
    def __init__(self, rows, events):
        self.telemetry, self.events = rows, events

    def configure_compensator(self, *args, **kwargs):
        pass

    def run_scenario(self, **kwargs):
        return self.telemetry


@pytest.mark.parametrize("attitude,expected,censored", [(0, 0.0, False), (3, None, True)])
def test_legacy_evaluate_trial_uses_formal_recovery_without_1p2_default(attitude, expected, censored):
    from offline_learning.evaluate_splits import evaluate_trial
    rows = [row(0, attitude_deg=attitude), row(.4, mode="SEPARATING", attitude_deg=attitude),
            row(1.2, mode="RECOVERY", attitude_deg=attitude)]
    events = [{"event": "DOCK", "time": 0}, {"event": "UNDOCK", "time": .4}]
    runner = LegacyFakeRunner(rows, events)
    result = evaluate_trial(runner, "baseline", {"approach_offset": 0, "q1_init_y": -.6, "q2_init_y": .6})
    assert result["settling_time_s"] == expected
    assert result["right_censored"] is censored
    assert len(result["window_times"]) == 3  # Includes separating/recovery modes.
    assert result["event_metrics"]["schema_version"] == "event_metrics_v1"


def test_legacy_evaluate_trial_retains_missing_capture_classification():
    from offline_learning.evaluate_splits import evaluate_trial
    runner = LegacyFakeRunner([row(0, mode="APPROACH"), row(1, mode="ALIGN")], [])
    result = evaluate_trial(runner, "baseline", {"approach_offset": 0, "q1_init_y": -.6, "q2_init_y": .6})
    assert not result["success"]
    assert result["outcome"]["reason"] == "no_contact"
    assert result["settling_time_s"] is None
    assert result["window_times"] == []

