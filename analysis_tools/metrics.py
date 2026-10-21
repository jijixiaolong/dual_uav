"""Event-anchored metrics for the real, physics-rate MuJoCo telemetry.

No flight-mode filtering is performed.  Each occurrence gets its own fixed
window, including occurrences whose requested window extends past the recording.
Errors use the *active* reference logged by ``recorder.py``.  Squared control
integrals have units N^2 s and (N m)^2 s; they are not mechanical work in joules.

``evaluate_events(rows, events, config=None)`` is the public JSON-safe API.
Important configuration keys are ``window_s`` (default 1.2),
``event_windows_s`` (event-name -> seconds), ``expected_events`` (default DOCK
and UNDOCK), ``attitude_tolerance_deg`` (0.5),
``position_tolerance_m`` (0.02), ``rate_tolerance_deg_s`` (5),
``motor_min_N`` (0.05), and ``motor_max_N`` (5.5).  Motor bounds may be scalars
or eight-element sequences.  Unavailable quantities are None, never zero/NaN.
"""
from __future__ import annotations

from bisect import bisect_left
import math
from typing import Mapping

import numpy as np


EVENT_NAMES = ("CONTACT", "CAPTURE_READY", "DOCK", "UNDOCK", "INITIAL_CONNECTED")
DEFAULT_CONFIG = {
    "window_s": 1.2,
    "event_windows_s": {},
    "expected_events": ["DOCK", "UNDOCK"],
    "attitude_tolerance_deg": 0.5,
    "position_tolerance_m": 0.02,
    "rate_tolerance_deg_s": 5.0,
    "motor_min_N": 0.05,
    "motor_max_N": 5.5,
    "saturation_epsilon_N": 1e-6,
    "separation_success_distance_m": 0.05,
    "digital_signal_hold": "auto",
    "min_altitude_m": None,
    "max_tilt_deg": None,
    "max_speed_mps": None,
    "simulation_duration_s": None,
    "expected_end_time_s": None,
}


def _number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else math.nan
    except (TypeError, ValueError):
        return math.nan


def _clean(value):
    if isinstance(value, Mapping):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_clean(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def _array(rows, key):
    return np.asarray([_number(row.get(key)) for row in rows], dtype=float)


def _vectors(rows, prefix):
    return np.column_stack([_array(rows, f"{prefix}_{axis}") for axis in "xyz"])


def _all_finite(values):
    return bool(np.size(values) and np.all(np.isfinite(values)))


def _integral(times, values):
    if len(times) < 2 or not _all_finite(values):
        return None
    return float(np.sum(np.diff(times) * (values[:-1] + values[1:]) / 2.0))


def _square_integral(times, values):
    """Exact integral of the square of a piecewise linear sampled signal."""
    if len(times) < 2 or not _all_finite(values):
        return None
    values = np.asarray(values)
    if values.ndim == 1:
        values = values[:, None]
    a, b = values[:-1], values[1:]
    return float(np.sum(np.diff(times)[:, None] * (a*a + a*b + b*b) / 3.0))


def _maximum(values):
    return float(np.max(values)) if _all_finite(values) else None


def settling_time(times, normalized_errors):
    """Earliest time after which every normalized error stays <= 1.

    ``times`` starts at the available event-window boundary.  Crossing times
    are linearly interpolated in each error, with equality inside tolerance.
    A final out-of-tolerance sample is right censored, so time is None.  A
    truncated recording is represented separately by the caller; this function
    never converts an unrecovered trace into the nominal window length.
    """
    times = np.asarray(times, dtype=float)
    errors = np.asarray(normalized_errors, dtype=float)
    if errors.ndim == 1:
        errors = errors[:, None]
    if len(times) == 0 or errors.shape[0] != len(times) or not _all_finite(errors):
        return {"time_s": None, "right_censored": None, "status": "unavailable"}
    inside = np.all(errors <= 1.0 + 1e-12, axis=1)
    if not inside[-1]:
        return {"time_s": None, "right_censored": True, "status": "unrecovered"}
    outside = np.flatnonzero(~inside)
    if not len(outside):
        return {"time_s": 0.0, "right_censored": False, "status": "within_tolerance_from_start"}
    k = int(outside[-1])
    fractions = [(a-1.0)/(a-b) for a, b in zip(errors[k], errors[k+1]) if a > 1.0]
    fraction = max(fractions, default=0.0)
    recovered = times[k] + fraction*(times[k+1]-times[k]) - times[0]
    return {"time_s": float(recovered), "right_censored": False, "status": "recovered"}


def _slerp(a, b, fraction):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        result = a + fraction*(b-a)
        return result/np.linalg.norm(result)
    angle = math.acos(np.clip(dot, -1, 1))
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b)/math.sin(angle)


_DISCRETE = {"mode", "mode_name", "phase", "weld_active", "aggregate_controller_active",
             "control_updated", "connector_contact_count", "u_comp_active"}


def _hold_side(rows, config=None):
    side = (config or {}).get("digital_signal_hold", "auto")
    if side not in ("auto", "left", "right"):
        raise ValueError("digital_signal_hold must be auto, left, or right")
    if side == "auto":
        # recorder.py logs state *after* stepping: each row's digital input
        # belongs to the physical interval which has just ended at row.time.
        return "right" if rows and "controller_state_time_s" in rows[0] else "left"
    return side


def _interpolate_row(rows, times, time, hold_side="left"):
    index = bisect_left(times, time)
    if index < len(times) and abs(times[index]-time) < 1e-10:
        row = dict(rows[index])
        row["time"] = time
        return row
    if index == 0 or index == len(times):
        raise ValueError("Interpolation cannot extrapolate beyond recorded telemetry")
    left, right = rows[index-1], rows[index]
    fraction = (time-times[index-1])/(times[index]-times[index-1])
    row = dict(left)
    for key in left.keys() | right.keys():
        if (key in _DISCRETE or key == "time" or key.endswith("_time_s") or
                key.endswith(("_command_N", "_target_N", "_desired_thrust_N")) or
                "_desired_torque_Nm_" in key):
            if hold_side == "right":
                row[key] = right.get(key)
            continue
        a, b = _number(left.get(key)), _number(right.get(key))
        if math.isfinite(a) and math.isfinite(b):
            row[key] = a + fraction*(b-a)
    # Interpolate quaternion geometry on S^3, not component-wise.  Recorded
    # scalar errors are recomputed at the synthetic boundary when possible.
    for quad in ("q1", "q2"):
        for prefix in (f"{quad}_quat", f"{quad}_desired_quat"):
            a = [_number(left.get(f"{prefix}_{axis}")) for axis in "wxyz"]
            b = [_number(right.get(f"{prefix}_{axis}")) for axis in "wxyz"]
            if _all_finite(a) and _all_finite(b) and np.linalg.norm(a)*np.linalg.norm(b) > 0:
                row.update({f"{prefix}_{axis}": float(x) for axis, x in zip("wxyz", _slerp(a, b, fraction))})
        actual = np.asarray([_number(row.get(f"{quad}_quat_{axis}")) for axis in "wxyz"])
        desired = np.asarray([_number(row.get(f"{quad}_desired_quat_{axis}")) for axis in "wxyz"])
        if _all_finite(actual) and _all_finite(desired) and np.linalg.norm(actual)*np.linalg.norm(desired) > 0:
            actual, desired = actual/np.linalg.norm(actual), desired/np.linalg.norm(desired)
            row[f"{quad}_attitude_error_rad"] = float(2*math.acos(np.clip(abs(actual @ desired), 0, 1)))
            dw, dx, dy, dz = desired * np.asarray([1, -1, -1, -1])
            aw, ax, ay, az = actual
            w, x, y, z = (dw*aw-dx*ax-dy*ay-dz*az,
                          dw*ax+dx*aw+dy*az-dz*ay,
                          dw*ay-dx*az+dy*aw+dz*ax,
                          dw*az+dx*ay-dy*ax+dz*aw)
            row[f"{quad}_roll_error_deg"] = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
            row[f"{quad}_pitch_error_deg"] = math.degrees(math.asin(np.clip(2*(w*y-z*x), -1, 1)))
        position = np.asarray([_number(row.get(f"{quad}_{axis}")) for axis in "xyz"])
        reference = np.asarray([_number(row.get(f"{quad}_reference_{axis}")) for axis in "xyz"])
        if _all_finite(position) and _all_finite(reference):
            row[f"{quad}_pos_error_m"] = float(np.linalg.norm(position-reference))
    row["time"] = time
    row["control_updated"], row["control_dt_s"] = 0, 0.0
    return row


def event_window(rows, event_time, duration, hold_side="auto"):
    """Return boundary-interpolated rows and explicit coverage information."""
    if duration <= 0 or not math.isfinite(duration):
        raise ValueError("Event window duration must be positive and finite")
    stop = event_time + duration
    if not rows:
        return [], {"requested_start_s": event_time, "requested_end_s": stop,
                    "observed_start_s": None, "observed_end_s": None,
                    "observed_duration_s": 0.0, "complete": False,
                    "left_truncated": True, "right_truncated": True,
                    "status": "missing_telemetry", "sample_count": 0}
    times = [_number(row["time"]) for row in rows]
    hold_side = _hold_side(rows, {"digital_signal_hold": hold_side})
    first, last = times[0], times[-1]
    start, end = max(event_time, first), min(stop, last)
    left, right = first > event_time+1e-10, last < stop-1e-10
    metadata = {"requested_start_s": event_time, "requested_end_s": stop,
                "observed_start_s": start if start <= end else None,
                "observed_end_s": end if start <= end else None,
                "observed_duration_s": max(0.0, end-start), "complete": not (left or right),
                "left_truncated": left, "right_truncated": right,
                "status": "complete" if not (left or right) else "truncated",
                "sample_count": 0}
    if start > end:
        metadata["status"] = "outside_recording"
        return [], metadata
    selected = [_interpolate_row(rows, times, start, hold_side)]
    selected.extend(dict(row) for row, time in zip(rows, times) if start+1e-10 < time < end-1e-10)
    if end > start+1e-10:
        selected.append(_interpolate_row(rows, times, end, hold_side))
    metadata["sample_count"] = len(selected)
    return selected, metadata


def _quad_metrics(rows, times, quad, config):
    attitude = np.rad2deg(_array(rows, f"{quad}_attitude_error_rad"))
    roll = np.abs(_array(rows, f"{quad}_roll_error_deg"))
    pitch = np.abs(_array(rows, f"{quad}_pitch_error_deg"))
    omega = _vectors(rows, f"{quad}_omega_body")
    rate = np.rad2deg(np.linalg.norm(omega, axis=1))
    position = _array(rows, f"{quad}_pos_error_m")
    signals = np.column_stack([attitude/config["attitude_tolerance_deg"],
                               position/config["position_tolerance_m"],
                               rate/config["rate_tolerance_deg_s"]])
    result = {
        "peak_attitude_error_deg": _maximum(attitude),
        "peak_roll_error_deg": _maximum(roll), "peak_pitch_error_deg": _maximum(pitch),
        "peak_absolute_roll_deg": _maximum(np.abs(_array(rows, f"{quad}_roll_deg"))),
        "peak_absolute_pitch_deg": _maximum(np.abs(_array(rows, f"{quad}_pitch_deg"))),
        "peak_rate_deg_s": _maximum(rate),
        "peak_rate_rad_s": _maximum(np.deg2rad(rate)),
        "peak_body_rate_components_rad_s": {axis: _maximum(np.abs(omega[:, k])) for k, axis in enumerate("xyz")},
        "max_position_error_m": _maximum(position),
        "attitude_error_integral_deg_s": _integral(times, attitude),
        "position_error_integral_m_s": _integral(times, position),
        "attitude_error_squared_integral_deg2_s": _square_integral(times, attitude),
        "position_error_squared_integral_m2_s": _square_integral(times, position),
        "attitude_recovery": settling_time(times, signals[:, 0]),
        "position_recovery": settling_time(times, signals[:, 1]),
        "rate_recovery": settling_time(times, signals[:, 2]),
        "joint_recovery": settling_time(times, signals),
    }
    result["settling_time_s"] = result["joint_recovery"]["time_s"]
    result["right_censored"] = result["joint_recovery"]["right_censored"]
    return result, signals


def _zoh_integral(times, values, side="left"):
    if len(times) < 2 or not _all_finite(values):
        return None
    values = np.asarray(values)
    held = values[1:] if side == "right" else values[:-1]
    if values.ndim == 1:
        return float(np.sum(np.diff(times)*held))
    return float(np.sum(np.diff(times)[:, None]*held))


def _motor_metrics(rows, times, config):
    result = {}
    duration = float(times[-1]-times[0]) if len(times) else 0.0
    minimum = np.broadcast_to(np.asarray(config["motor_min_N"], dtype=float), (8,))
    maximum = np.broadcast_to(np.asarray(config["motor_max_N"], dtype=float), (8,))
    epsilon = float(config["saturation_epsilon_N"])
    hold_side = _hold_side(rows, config)
    for kind in ("command", "target", "actual"):
        motors = np.column_stack([_array(rows, f"motor{k}_{kind}_N") for k in range(8)])
        valid = _all_finite(motors) and len(times) > 1
        if not valid:
            result[kind] = {"available": False}
            continue
        delta = np.diff(motors, axis=0)
        slope = delta/np.diff(times)[:, None]
        sat = ((motors <= minimum+epsilon) | (motors >= maximum-epsilon)).astype(float)
        # Command and target are held digital signals. Actual motor state is
        # continuous; occupancy of its linear segment is integrated exactly.
        if kind in ("command", "target"):
            squared = _zoh_integral(times, motors*motors, hold_side)
            sat_held = sat[1:] if hold_side == "right" else sat[:-1]
            sat_per_motor = np.sum(np.diff(times)[:, None]*sat_held, axis=0)/duration
        else:
            squared = _square_integral(times, motors)
            occupancy = np.empty_like(delta)
            for n in range(len(delta)):
                for k in range(8):
                    a, b = motors[n, k], motors[n+1, k]
                    points = [0.0, 1.0]
                    if a != b:
                        for bound in (minimum[k]+epsilon, maximum[k]-epsilon):
                            crossing = (bound-a)/(b-a)
                            if 0 < crossing < 1:
                                points.append(float(crossing))
                    points.sort()
                    occupancy[n, k] = sum(v-u for u, v in zip(points[:-1], points[1:])
                        if a+(b-a)*(u+v)/2 <= minimum[k]+epsilon or
                           a+(b-a)*(u+v)/2 >= maximum[k]-epsilon)
            sat_per_motor = np.sum(np.diff(times)[:, None]*occupancy, axis=0)/duration
        result[kind] = {
            "available": True, "thrust_squared_integral_N2_s": squared,
            "total_variation_N": float(np.sum(np.abs(delta))),
            "peak_sample_slope_N_s": float(np.max(np.abs(slope))),
            "mean_absolute_slope_N_s": float(np.sum(np.abs(delta))/(8*duration)),
            "slope_squared_integral_N2_per_s": float(np.sum(np.diff(times)[:, None]*slope*slope)),
            "saturation_fraction": float(np.mean(sat_per_motor)),
            "saturation_fraction_per_motor": sat_per_motor.tolist(),
            "sample_hold": f"{hold_side}_zero_order" if kind != "actual" else "piecewise_linear",
        }
        if kind == "command":
            # This is the commanded slew at each controller update; the above
            # physical-sample slope describes the sampled held waveform.
            update_slopes = []
            for n in range(1, len(rows)):
                dt = _number(rows[n].get("control_dt_s"))
                if _number(rows[n].get("control_updated")) == 1 and dt > 0:
                    update_slopes.extend((delta[n-1]/dt).tolist())
            result[kind]["peak_control_update_slope_N_s"] = _maximum(np.abs(update_slopes))
    return result


def _control_metrics(rows, times, config):
    # Select the active desired wrench before integration.  Recorder keeps the
    # last values from inactive controllers, which must not be summed in.
    thrusts, torques = [], []
    for row in rows:
        aggregate = _number(row.get("aggregate_controller_active", row.get("weld_active", 0))) == 1
        if aggregate:
            thrusts.append([_number(row.get("aggregate_desired_thrust_N")), 0.0])
            torques.append([_number(row.get(f"aggregate_desired_torque_Nm_{axis}")) for axis in "xyz"] + [0.0]*3)
        else:
            thrusts.append([_number(row.get(f"q{i}_desired_thrust_N")) for i in (1, 2)])
            torques.append([_number(row.get(f"q{i}_desired_torque_Nm_{axis}")) for i in (1, 2) for axis in "xyz"])
    thrusts, torques = np.asarray(thrusts), np.asarray(torques)
    actual_forces = np.column_stack([_vectors(rows, f"q{i}_rotor_force_world") for i in (1, 2)])
    actual_torques = np.column_stack([_vectors(rows, f"q{i}_rotor_torque_body") for i in (1, 2)])
    hold_side = _hold_side(rows, config)
    return {
        "active_desired_thrust_squared_integral_N2_s": _zoh_integral(times, thrusts*thrusts, hold_side),
        "active_desired_torque_squared_integral_Nm2_s": _zoh_integral(times, torques*torques, hold_side),
        "actual_rotor_force_squared_integral_N2_s": _square_integral(times, actual_forces),
        "actual_rotor_torque_squared_integral_Nm2_s": _square_integral(times, actual_torques),
        "desired_wrench_scope": "aggregate wrench while connected; sum of two per-quad squared wrenches while independent",
        "actual_wrench_scope": "sum of two per-quad squared physical rotor wrenches; comparable across modes",
        "energy_is_mechanical_work_J": False,
        "digital_signal_hold": hold_side,
    }


def _flight_envelope(rows, config):
    minima, tilts, speeds, violations = [], [], [], []
    for row in rows:
        for quad in ("q1", "q2"):
            altitude = _number(row.get(f"{quad}_z"))
            if math.isfinite(altitude):
                minima.append(altitude)
                if config["min_altitude_m"] is not None and altitude < config["min_altitude_m"]:
                    violations.append({"time_s": _number(row["time"]), "vehicle": quad,
                                       "quantity": "altitude_m", "value": altitude,
                                       "limit": config["min_altitude_m"]})
            quaternion = np.asarray([_number(row.get(f"{quad}_quat_{a}")) for a in "wxyz"])
            tilt = math.nan
            if _all_finite(quaternion) and np.linalg.norm(quaternion) > 0:
                w, x, y, z = quaternion/np.linalg.norm(quaternion)
                tilt = math.degrees(math.acos(np.clip(1-2*(x*x+y*y), -1, 1)))
            else:
                roll, pitch = _number(row.get(f"{quad}_roll_deg")), _number(row.get(f"{quad}_pitch_deg"))
                if math.isfinite(roll) and math.isfinite(pitch):
                    tilt = math.degrees(math.acos(np.clip(math.cos(math.radians(roll))*math.cos(math.radians(pitch)), -1, 1)))
            if math.isfinite(tilt):
                tilts.append(tilt)
                if config["max_tilt_deg"] is not None and tilt > config["max_tilt_deg"]:
                    violations.append({"time_s": _number(row["time"]), "vehicle": quad,
                                       "quantity": "tilt_deg", "value": tilt,
                                       "limit": config["max_tilt_deg"]})
            velocity = [_number(row.get(f"{quad}_velocity_{a}")) for a in "xyz"]
            if _all_finite(velocity):
                speed = float(np.linalg.norm(velocity))
                speeds.append(speed)
                if config["max_speed_mps"] is not None and speed > config["max_speed_mps"]:
                    violations.append({"time_s": _number(row["time"]), "vehicle": quad,
                                       "quantity": "speed_mps", "value": speed,
                                       "limit": config["max_speed_mps"]})
    return {"violation": bool(violations), "first_violation": violations[0] if violations else None,
            "violating_vehicle_sample_count": len(violations),
            "minimum_observed_altitude_m": min(minima) if minima else None,
            "maximum_observed_tilt_deg": max(tilts) if tilts else None,
            "maximum_observed_speed_mps": max(speeds) if speeds else None}


def _outcome(rows, events, config):
    names = {str(event.get("event", event.get("type", ""))).upper() for event in events}
    modes = {str(row.get("mode_name", "")).upper() for row in rows}
    expected = set(config["expected_events"])
    docked = bool(names & {"DOCK", "INITIAL_CONNECTED"})
    released = "UNDOCK" in names
    aborted = "ABORT" in names or "ABORT" in modes
    final = rows[-1] if rows else {}
    envelope = _flight_envelope(rows, config)
    expected_end = config["expected_end_time_s"]
    if expected_end is None:
        expected_end = config["simulation_duration_s"]
    prematurely_ended = bool(rows and expected_end is not None and
                             _number(final.get("time")) < float(expected_end)-1e-6)
    actual_state_fields = {f"q{i}_{axis}" for i in (1, 2) for axis in "xyz"}
    actual_state_fields |= {f"q{i}_quat_{axis}" for i in (1, 2) for axis in "wxyz"}
    actual_state_fields |= {f"q{i}_omega_body_{axis}" for i in (1, 2) for axis in "xyz"}
    finite_state = all(math.isfinite(_number(value)) for row in rows
                       for key, value in row.items() if key == "time" or key in actual_state_fields)
    observed_release_time = next((_number(event.get("time", event.get("time_s"))) for event in events
                                  if event.get("event", event.get("type")) == "UNDOCK"), math.inf)
    separated = released and any(_number(row.get("time")) >= observed_release_time and
        _number(row.get("weld_active", 1)) == 0 and
        _number(row.get("rel_dist")) >= config["separation_success_distance_m"] for row in rows)
    if not rows:
        category, reason = "failure", "missing_telemetry"
    elif aborted:
        category, reason = "aborted", "fsm_abort"
    elif not finite_state:
        category, reason = "failure", "nonfinite_state"
    elif envelope["violation"]:
        category, reason = "failure", "flight_envelope_violation"
    elif prematurely_ended:
        category, reason = "timeout", "observation_ended_early"
    elif "DOCK" in expected and not docked:
        category, reason = "timeout", "capture_timeout" if "CONTACT" in names else "no_contact"
    elif "UNDOCK" in expected and not released:
        category, reason = "timeout", "release_not_observed"
    elif "UNDOCK" in expected and not separated:
        category, reason = "timeout", "release_without_observed_separation"
    elif released and _number(final.get("weld_active")) == 1:
        category, reason = "failure", "weld_reactivated_after_release"
    else:
        category, reason = "success", "expected_events_completed"
    return {"category": category, "reason": reason,
            "docking_success": docked and not aborted,
            "release_observed": released, "separation_success": bool(separated and not aborted and
                finite_state and not envelope["violation"]),
            "aborted": aborted, "timed_out": category == "timeout",
            "failed": category in ("failure", "aborted", "timeout"),
            "expected_events": sorted(expected), "flight_envelope": envelope,
            "observation_ended_early": prematurely_ended,
            "expected_end_time_s": expected_end}


def _prepare(rows, config):
    cfg = {**DEFAULT_CONFIG, **(config or {})}
    cfg["event_windows_s"] = {**DEFAULT_CONFIG["event_windows_s"], **cfg["event_windows_s"]}
    cfg["expected_events"] = [str(name).upper() for name in cfg["expected_events"]]
    for key in ("attitude_tolerance_deg", "position_tolerance_m", "rate_tolerance_deg_s"):
        if not math.isfinite(float(cfg[key])) or float(cfg[key]) <= 0:
            raise ValueError(f"{key} must be positive and finite")
    for key in ("min_altitude_m", "max_tilt_deg", "max_speed_mps", "simulation_duration_s", "expected_end_time_s"):
        if cfg[key] is not None:
            cfg[key] = float(cfg[key])
            if not math.isfinite(cfg[key]):
                raise ValueError(f"{key} must be finite when specified")
            if key in ("max_tilt_deg", "max_speed_mps", "simulation_duration_s") and cfg[key] <= 0:
                raise ValueError(f"{key} must be positive when specified")
    minimum = np.broadcast_to(np.asarray(cfg["motor_min_N"], dtype=float), (8,))
    maximum = np.broadcast_to(np.asarray(cfg["motor_max_N"], dtype=float), (8,))
    if not _all_finite(minimum) or not _all_finite(maximum) or np.any(maximum <= minimum):
        raise ValueError("Eight motor bounds must be finite and minimum < maximum")
    _hold_side([], cfg)
    rows = list(rows)
    times = [_number(row.get("time")) for row in rows]
    if not all(math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times[:-1], times[1:])):
        raise ValueError("Telemetry timestamps must be finite and strictly increasing")
    return rows, cfg


def _window_metrics(window, metadata, cfg):
    if not window:
        return None
    t = np.asarray([_number(row["time"]) for row in window])
    quads, signals = {}, []
    for quad in ("q1", "q2"):
        quads[quad], signal = _quad_metrics(window, t, quad, cfg)
        signals.append(signal)
        for key in ("attitude_recovery", "position_recovery", "rate_recovery", "joint_recovery"):
            # Left truncation destroys the event-relative settling-time
            # estimate even when the observed suffix is inside tolerance.
            if metadata["left_truncated"]:
                quads[quad][key] = {"time_s": None, "right_censored": None,
                                    "status": "left_truncated"}
        quads[quad]["settling_time_s"] = quads[quad]["joint_recovery"]["time_s"]
        quads[quad]["right_censored"] = quads[quad]["joint_recovery"]["right_censored"]
    combined_recovery = settling_time(t, np.column_stack(signals))
    if metadata["left_truncated"]:
        combined_recovery = {"time_s": None, "right_censored": None, "status": "left_truncated"}
    combined = {
        key: max(quads[q][key] for q in ("q1", "q2"))
        if all(quads[q][key] is not None for q in ("q1", "q2")) else None
        for key in ("peak_attitude_error_deg", "peak_roll_error_deg", "peak_pitch_error_deg",
                    "peak_rate_deg_s", "peak_rate_rad_s", "max_position_error_m")}
    for key in ("attitude_error_integral_deg_s", "position_error_integral_m_s",
                "attitude_error_squared_integral_deg2_s", "position_error_squared_integral_m2_s"):
        combined[key] = sum(quads[q][key] for q in ("q1", "q2")) if all(quads[q][key] is not None for q in ("q1", "q2")) else None
    combined.update(joint_recovery=combined_recovery, settling_time_s=combined_recovery["time_s"],
                    right_censored=combined_recovery["right_censored"])
    return {**quads, "combined": combined,
                          "control": _control_metrics(window, t, cfg),
                          "motors": _motor_metrics(window, t, cfg),
                          "peak_contact_force_N": _maximum(_array(window, "contact_normal_force_N")),
                          "contact_force_integral_N_s": _integral(t, _array(window, "contact_normal_force_N")),
                          "peak_weld_force_N": _maximum(np.linalg.norm(_vectors(window, "weld_q1_force_world_N"), axis=1))}


def evaluate_window(rows, anchor_time_s, duration_s=1.2, config=None):
    """Evaluate a declared intervention window without inventing flight events.

    In particular, a control retaining its weld can use the paired release
    experiment's intervention time as its anchor without claiming an UNDOCK.
    The returned window/metrics have exactly the schema used by event records.
    """
    rows, cfg = _prepare(rows, config)
    anchor = _number(anchor_time_s)
    if not math.isfinite(anchor):
        raise ValueError("Window anchor must be finite")
    window, metadata = event_window(rows, anchor, float(duration_s), cfg["digital_signal_hold"])
    return _clean({"anchor_time_s": anchor, "window": metadata,
                   "metrics": _window_metrics(window, metadata, cfg)})


def evaluate_events(rows, events, config=None):
    """Evaluate every occurrence and retain absent-event and failure records.

    Input rows may be dictionaries read directly from csv.DictReader.  Event
    dictionaries use ``event`` and ``time`` (``type``/``time_s`` accepted).
    Output can be serialized with json.dumps(..., allow_nan=False).
    """
    rows, cfg = _prepare(rows, config)
    events = list(events)
    output = []
    counts = {name: 0 for name in EVENT_NAMES}
    for event in events:
        name = str(event.get("event", event.get("type", ""))).upper()
        if name not in EVENT_NAMES:
            continue
        t0 = _number(event.get("time", event.get("time_s")))
        if not math.isfinite(t0):
            raise ValueError(f"Nonfinite timestamp for {name}")
        index = counts[name]
        counts[name] += 1
        duration = float(cfg["event_windows_s"].get(name, cfg["window_s"]))
        window, metadata = event_window(rows, t0, duration, cfg["digital_signal_hold"])
        record = {"event": name, "occurrence": index, "occurred": True, "event_time_s": t0,
                  "window": metadata, "metrics": None}
        record["metrics"] = _window_metrics(window, metadata, cfg)
        output.append(record)
    for name in EVENT_NAMES:
        if counts[name] == 0:
            classification = "required_event_missing" if name in cfg["expected_events"] else "not_observed"
            if name == "DOCK" and counts["INITIAL_CONNECTED"]:
                classification = "satisfied_by_initial_connection"
            output.append({"event": name, "occurrence": None, "occurred": False,
                           "event_time_s": None, "window": None, "metrics": None,
                           "classification": classification})
    return _clean({"schema_version": "event_metrics_v1", "config": cfg,
                   "definitions": {"settling": "first time all specified errors stay within tolerance to observed window end; no dwell padding",
                                   "window": "exact event time, fixed duration, interpolated boundaries, no flight-mode filter",
                                   "attitude": "SO(3) geodesic error to active desired attitude; degrees",
                                   "joint_recovery": "both vehicles: attitude, position, and angular-rate tolerances simultaneously",
                                   "motor_saturation": "time-weighted fraction of eight motor-seconds at either configured bound",
                                   "squared_control_units": "thrust: N^2 s; torque: (N m)^2 s; neither is energy in J",
                                   "digital_signal_hold": "auto: post-step recorder inputs use right ZOH; generic rows use left ZOH; actual motors use linear interpolation",
                                   "truncated_recovery": "recovery refers only to observed suffix; window.complete must be checked before comparison"},
                   "outcome": _outcome(rows, events, cfg), "events": output})
