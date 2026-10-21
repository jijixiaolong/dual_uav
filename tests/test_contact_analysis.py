import math

import pytest

from analysis_tools.contact_analysis import capture_sample, contiguous_intervals, reconstruct_control_ticks


LIMITS = dict(position_tolerance_m=.02, relative_linear_velocity_mps=.1,
              orientation_tolerance_rad=.08, relative_angular_velocity_radps=.2,
              dwell_time_s=.03)


def row(t=0., mode='ALIGN', **changes):
    result = dict(time=t, controller_state_time_s=t-.002, control_updated=0,
                  control_dt_s=0., mode_name=mode, relative_attitude_rad=.08,
                  connector_contact_count=2, contact_normal_force_N=0.)
    for prefix, x in (('connector_relative_position', .02),
                      ('connector_relative_velocity', .1),
                      ('connector_relative_angular_velocity_world', .2)):
        for axis in 'xyz':
            result[prefix+'_'+axis] = x if axis == 'x' else 0.
    result.update(changes)
    return result


def test_capture_inclusive_boundaries_and_zero_force_contact():
    sample = capture_sample(row(), LIMITS)
    assert sample['capture_eligible']
    assert sample['radial_speed_mps'] == pytest.approx(.1)
    assert sample['contact_normal_force_N'] == 0.


@pytest.mark.parametrize('key,value,guard', [
    ('connector_contact_count', 0, 'pass_contact'),
    ('connector_relative_position_x', .02001, 'pass_position'),
    ('connector_relative_velocity_x', .10001, 'pass_linear_speed'),
    ('relative_attitude_rad', .08001, 'pass_orientation'),
    ('connector_relative_angular_velocity_world_x', .20001, 'pass_angular_speed'),
    ('connector_relative_velocity_x', math.nan, 'pass_linear_speed'),
])
def test_capture_rejects_every_guard_and_nonfinite(key, value, guard):
    sample = capture_sample(row(**{key: value}), LIMITS)
    assert not sample[guard]
    assert not sample['capture_eligible']


def test_contact_intervals_preserve_gap_and_censored_end():
    result = contiguous_intervals([0., .002, .004, .006, .008], [True, True, False, False, True], .010)
    assert len(result) == 2
    assert result[0]['duration_s'] == .004
    assert not result[0]['right_censored']
    assert result[1]['duration_s'] == .002
    assert result[1]['right_censored']
    with pytest.raises(ValueError):
        contiguous_intervals([0., 0.], [True, True])


def test_replay_uses_preceding_input_and_resets_dwell_when_contact_lost():
    # Initialize FSM mode through the first record, then enter capture using
    # t=.002 state. Its t=.004 output has already lost contact.
    rows = [row(.002, control_updated=1, controller_state_time_s=0., mode='ALIGN'),
            row(.004, mode='CAPTURE_PENDING', control_updated=1, control_dt_s=.002,
                connector_contact_count=0),
            row(.006, mode='ALIGN', control_updated=1, control_dt_s=.002)]
    ticks = reconstruct_control_ticks(rows, LIMITS)
    assert ticks[1]['capture_eligible']
    assert ticks[1]['capture_guard_replay_matches']
    assert not ticks[2]['capture_eligible']
    assert ticks[2]['dwell_reset']
    assert ticks[2]['dwell_after_s'] == 0.
    assert ticks[2]['capture_guard_replay_matches']


def test_replay_preserves_actual_floating_point_dwell_delay():
    # Real MuJoCo accumulated times at the 4/6 ms control ticks of this capture.
    times = [4.869999999999686, 4.875999999999685, 4.879999999999685,
             4.885999999999684, 4.8899999999996835, 4.895999999999683,
             4.899999999999682, 4.905999999999682]
    rows = [row(times[0]-.002, mode='ALIGN', control_updated=1,
                controller_state_time_s=times[0]-.004)]
    rows.append(row(times[0], mode='ALIGN'))
    for i, time in enumerate(times):
        mode = 'CONNECTED' if i == len(times)-1 else 'CAPTURE_PENDING'
        rows.append(row(time+.002, mode=mode, control_updated=1,
                        controller_state_time_s=time,
                        control_dt_s=time-times[i-1] if i else .004))
        if i+1 < len(times):
            rows.append(row(times[i+1], mode=mode))
    ticks = reconstruct_control_ticks(rows, LIMITS)
    assert ticks[-2]['dwell_after_s'] < LIMITS['dwell_time_s']
    assert ticks[-2]['dwell_after_s'] == pytest.approx(.03)
    assert ticks[-1]['dwell_after_s'] == pytest.approx(.036)
    assert all(t['capture_guard_replay_matches'] for t in ticks)
