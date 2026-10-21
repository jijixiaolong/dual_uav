"""Read-only reconstruction of the P0/P1 contact/capture timeline.

Telemetry is recorded *after* each physics step. A row marked control_updated
contains the FSM result at controller_state_time_s, but its state is 2 ms newer.
The preceding physics row is therefore the exact FSM input, not this row.
"""
import math


AXES = 'xyz'


def number(row, key):
    return float(row[key])


def vector(row, prefix):
    return [number(row, prefix + '_' + axis) for axis in AXES]


def norm(values):
    return math.sqrt(sum(x*x for x in values))


def desired_pair_angle(row):
    q1 = [number(row, f'q1_desired_quat_{c}') for c in 'wxyz']
    q2 = [number(row, f'q2_desired_quat_{c}') for c in 'wxyz']
    dot = abs(sum(x*y for x, y in zip(q1, q2))) / (norm(q1)*norm(q2))
    return 2*math.acos(min(1., max(0., dot)))


def capture_sample(row, limits):
    """Same inclusive physical guards as LatchManager/HybridFSM; no relaxation."""
    p = vector(row, 'connector_relative_position')
    v = vector(row, 'connector_relative_velocity')
    w = vector(row, 'connector_relative_angular_velocity_world')
    distance, speed, omega = norm(p), norm(v), norm(w)
    angle = number(row, 'relative_attitude_rad')
    values = (distance, speed, angle, omega)
    finite = all(math.isfinite(x) for x in values)
    result = dict(time_s=number(row, 'time'), distance_m=distance,
                  relative_speed_mps=speed, relative_angle_rad=angle,
                  relative_angular_speed_radps=omega,
                  radial_speed_mps=sum(x*y for x, y in zip(p, v))/distance if distance > 0 else 0.,
                  contact_count=int(number(row, 'connector_contact_count')),
                  contact_normal_force_N=number(row, 'contact_normal_force_N'),
                  finite_state=finite,
                  pass_contact=number(row, 'connector_contact_count') > 0,
                  pass_position=finite and distance <= limits['position_tolerance_m'],
                  pass_linear_speed=finite and speed <= limits['relative_linear_velocity_mps'],
                  pass_orientation=finite and angle <= limits['orientation_tolerance_rad'],
                  pass_angular_speed=finite and omega <= limits['relative_angular_velocity_radps'])
    result['pass_kinematics'] = all(result[k] for k in (
        'pass_position', 'pass_linear_speed', 'pass_orientation', 'pass_angular_speed'))
    result['capture_eligible'] = result['pass_contact'] and result['pass_kinematics']
    result['failed_guards'] = '|'.join(k[5:] for k in (
        'pass_contact', 'pass_position', 'pass_linear_speed', 'pass_orientation', 'pass_angular_speed')
        if not result[k])
    return result


def contiguous_intervals(times, mask, end_time=None):
    """Half-open sampled intervals; keep last sample and censoring explicit."""
    if len(times) != len(mask):
        raise ValueError('times and mask lengths differ')
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('times must increase strictly')
    if not times:
        return []
    if end_time is None:
        if len(times) < 2:
            raise ValueError('end_time is required for a single sample')
        end_time = times[-1] + (times[-1]-times[-2])
    if end_time < times[-1]:
        raise ValueError('end_time precedes last sample')
    result, start = [], None
    for i in range(len(times)+1):
        active = bool(mask[i]) if i < len(times) else False
        if active and start is None:
            start = i
        elif not active and start is not None:
            end = times[i] if i < len(times) else end_time
            result.append(dict(start_time_s=times[start], last_active_sample_time_s=times[i-1],
                               end_time_exclusive_s=end, duration_s=end-times[start],
                               samples=i-start, right_censored=i == len(times)))
            start = None
    return result


def reconstruct_control_ticks(rows, limits):
    """Replay only capture guards/dwell from logged inputs and actual elapsed dt.

    This does not simulate mechanics or alter the FSM. It deliberately preserves
    binary floating-point arithmetic to explain the existing 30->36 ms dwell.
    """
    result, dwell, previous_mode = [], 0., 'FREE'
    for i, output in enumerate(rows):
        if not int(number(output, 'control_updated')):
            continue
        mode = output['mode_name']
        t = number(output, 'controller_state_time_s')
        dt = number(output, 'control_dt_s')
        sample = capture_sample(rows[i-1], limits) if i else None
        if sample is not None and not math.isclose(sample['time_s'], t, abs_tol=1e-9):
            raise ValueError('Missing exact preceding physics sample for control input')
        eligible = bool(sample is not None and sample['capture_eligible'])
        before, reset, expected = dwell, False, None
        if previous_mode == 'ALIGN':
            expected = 'CAPTURE_PENDING' if eligible else ('APPROACH' if sample['distance_m'] > .30 else 'ALIGN')
            if eligible:
                dwell = 0.
        elif previous_mode == 'CAPTURE_PENDING':
            if eligible:
                dwell += dt
                expected = 'CONNECTED' if dwell >= limits['dwell_time_s'] else 'CAPTURE_PENDING'
            else:
                dwell, reset, expected = 0., True, 'ALIGN'
        entry = dict(controller_time_s=t, telemetry_time_s=number(output, 'time'),
                     input_state_time_s=sample['time_s'] if sample else None,
                     input_exact_available=sample is not None,
                     control_dt_s=dt, previous_mode=previous_mode, mode=mode,
                     dwell_before_s=before, dwell_after_s=dwell, dwell_reset=reset,
                     capture_guard_expected_mode=expected,
                     capture_guard_replay_matches=expected is None or expected == mode)
        if sample:
            entry.update({k: v for k, v in sample.items() if k != 'time_s'})
        result.append(entry)
        previous_mode = mode
    return result
