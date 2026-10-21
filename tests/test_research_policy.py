"""Event policy contracts, causal features, and actual-motor observer inputs."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mujoco_sim.python.sim_runner import SimulationRunner, v3
from mujoco_sim.python.research_policy import (
    DECAY_TIMES, EVENTS, FEATURE_NAMES, MAX_MOTOR_DELTA_N, ResearchPolicy,
    event_features, expand_roll_coefficients,
)


def fixture_runner(preconnected=False):
    runner = SimulationRunner()
    runner._reset(-.22 if preconnected else -.6, .22 if preconnected else .6, preconnected)
    runner.research_options = {}
    return runner


def test_roll_basis_shape_geometry_and_strict_coefficient_validation():
    coefficients = np.arange(8.)
    expanded = expand_roll_coefficients(coefficients)
    assert expanded.shape == (8, 4)
    np.testing.assert_allclose(expanded[:4].sum(axis=0), 0.)
    np.testing.assert_allclose(expanded[4:].sum(axis=0), 0.)
    np.testing.assert_allclose(expanded[0], coefficients[:4])
    np.testing.assert_allclose(expanded[6], -coefficients[4:])
    runner = fixture_runner()
    wrench = runner.alloc_q1.achieved_wrench([1., 1., -1., -1.])
    assert wrench.thrust == 0. and wrench.torque.x > 0.
    assert abs(wrench.torque.y) < 1e-14 and abs(wrench.torque.z) < 1e-14
    for invalid in ([0.] * 7, [0.] * 9, np.zeros((2, 4)), [np.nan] * 8):
        with pytest.raises(ValueError):
            expand_roll_coefficients(invalid)


def test_schema_exact_names_indices_and_no_weld_oracle_inputs():
    runner = fixture_runner()
    runner.research_options['control_delay_s'] = .012
    assert len(FEATURE_NAMES) == len(set(FEATURE_NAMES)) == 32
    values = event_features(runner, 'CONTACT')
    assert values.shape == (32,)
    np.testing.assert_allclose(values[15:23], runner.world.applied_thrusts)
    assert values[29] == runner.world.motor_tau and values[30] == .012 and values[31] == 1.
    runner.latch.weld_generalized_force = lambda: np.full(runner.world.model.nv, 1e8)
    np.testing.assert_array_equal(values, event_features(runner, 'DOCK'))
    with pytest.raises(ValueError):
        event_features(runner, 'CAPTURE_READY')
    runner.world.applied_thrusts[0] = np.nan
    with pytest.raises(ValueError):
        event_features(runner, 'UNDOCK')


def test_distinct_event_profiles_overlap_and_share_final_compensation_limit():
    runner = fixture_runner()
    policy = ResearchPolicy('B1', coefficients={event: np.full(8, 100.) for event in EVENTS})
    policy.reset(runner)
    policy.on_event(runner, 'CONTACT', 1., runner.world.applied_thrusts.copy())
    policy.on_event(runner, 'DOCK', 1.05, runner.world.applied_thrusts.copy())
    policy.on_event(runner, 'UNDOCK', 1.10, runner.world.applied_thrusts.copy())
    assert all(policy.feedforwards[event].is_active(1.2) for event in EVENTS)
    individual = [np.asarray(policy.feedforwards[event].evaluate(1.2)) for event in EVENTS]
    assert np.max(np.abs(sum(individual))) > MAX_MOTOR_DELTA_N
    delta = policy.motor_delta(runner, 1.2)
    assert np.max(np.abs(delta)) == pytest.approx(MAX_MOTOR_DELTA_N)
    assert delta[:4].sum() == pytest.approx(0., abs=1e-14)
    assert delta[4:].sum() == pytest.approx(0., abs=1e-14)
    assert set(policy.pre_event_packets) == set(EVENTS)
    assert policy.pre_event_packets['DOCK']['time'] == 1.05


def test_event_specific_coefficients_and_selector_strict_features():
    runner = fixture_runner()
    received = []
    def selector(event, values):
        received.append((event, values.copy()))
        return np.ones(8) * (1. if event == 'CONTACT' else -1.)
    policy = ResearchPolicy('B4', selector=selector)
    policy.reset(runner)
    policy.on_event(runner, 'CONTACT', 1., runner.world.applied_thrusts)
    policy.on_event(runner, 'DOCK', 3., runner.world.applied_thrusts)
    assert [item[0] for item in received] == ['CONTACT', 'DOCK']
    assert all(item[1].shape == (32,) for item in received)
    assert policy.motor_delta(runner, 1.2)[0] > 0.
    assert policy.motor_delta(runner, 3.2)[0] < 0.
    with pytest.raises(ValueError):
        ResearchPolicy('B4')
    with pytest.raises(ValueError):
        ResearchPolicy('B1', coefficients={'CAPTURE_READY': np.zeros(8)})
    invalid = ResearchPolicy('B4', selector=lambda event, values: np.zeros(9))
    invalid.reset(runner)
    with pytest.raises(ValueError):
        invalid.on_event(runner, 'DOCK', 1., runner.world.applied_thrusts)


def test_b0_collects_packets_without_compensation_and_reset_clears_history():
    runner = fixture_runner()
    policy = ResearchPolicy('B0')
    policy.reset(runner)
    policy.on_event(runner, 'CONTACT', 1., runner.world.applied_thrusts)
    np.testing.assert_array_equal(policy.motor_delta(runner, 1.2), np.zeros(8))
    assert policy.pre_event_packets['CONTACT']['schema_version'] == 1
    assert 'oracle_loads' in policy.pre_event_packets['CONTACT']
    policy.reset(runner)
    assert policy.pre_event_packets == {}


def test_dob_uses_interval_average_actual_motor_torque_without_constraint_oracle():
    runner = fixture_runner()
    policy = ResearchPolicy('B3')
    policy.reset(runner)
    # Observer hooks themselves must never read constraint or contact forces.
    def forbidden():
        raise AssertionError('Constraint oracle reached by disturbance observer')
    runner.latch.weld_generalized_force = forbidden
    runner.latch.check_contacts = forbidden
    policy.begin_control(runner, 0., .005)
    assert policy._observers['q1'].initialized
    moments = []
    for thrusts in ([3., 3., 2., 2., 2.5, 2.5, 2.5, 2.5],
                    [2.8, 2.8, 2.2, 2.2, 2.5, 2.5, 2.5, 2.5]):
        runner.world.applied_thrusts[:] = thrusts
        moments.append(np.array([runner.alloc_q1.achieved_wrench(thrusts[:4]).torque.x, 0., 0.]))
        policy.after_step(runner)
    average = np.mean(moments, axis=0)
    np.testing.assert_allclose(policy._moment_integrals['q1'] / policy._moment_elapsed_s,
                               average, atol=1e-14)
    # Zero measured acceleration despite rotor roll moment implies an opposing
    # external moment; compensation must have the positive rotor-moment sign.
    policy.begin_control(runner, .004, .004)
    correction = policy._compensations['q1']
    assert correction[0] > 0. and np.linalg.norm(correction) <= .03 + 1e-14
    assert policy._last_observer_interval_s == pytest.approx(.004)
    nominal = runner.last_wrenches['q1']
    nominal.thrust, nominal.torque = 10., v3([.01, .02, .03])
    corrected = policy.correct_wrench(runner, 'q1', nominal, .004, .004)
    assert corrected.thrust == 10.
    assert nominal.torque.x == .01
    assert corrected.torque.x > nominal.torque.x


def test_contact_preserves_dob_but_dock_undock_reset_and_mode_geometry_prime():
    runner = fixture_runner()
    policy = ResearchPolicy('B3')
    policy.reset(runner)
    policy.begin_control(runner, 0., .005)
    policy.on_event(runner, 'CONTACT', .1, runner.world.applied_thrusts)
    assert policy._observers['q1'].initialized
    policy.on_event(runner, 'DOCK', .2, runner.world.applied_thrusts)
    assert not policy._observers['q1'].initialized
    runner.aggregate_control_active = True
    policy.begin_control(runner, .2, .005)
    assert policy._observers['aggregate'].initialized
    np.testing.assert_array_equal(policy._compensations['aggregate'], np.zeros(3))
    policy.on_event(runner, 'UNDOCK', .3, runner.world.applied_thrusts)
    assert not policy._observers['aggregate'].initialized
    runner.aggregate_control_active = False
    policy.begin_control(runner, .3, .005)
    assert policy._observers['q1'].initialized
    np.testing.assert_array_equal(policy._compensations['q1'], np.zeros(3))


def test_b0_and_dob_full_runner_smoke_keep_common_motor_limits():
    for method in ('B0', 'B3'):
        runner = SimulationRunner()
        policy = ResearchPolicy(method)
        rows = runner.run_scenario(duration=.04, dock_cmd_time=10., undock_cmd_time=None,
                                   controller_policy=policy)
        assert len(rows) == 20
        assert np.isfinite([row['q1_attitude_error_rad'] for row in rows]).all()
        assert max(row[f'motor{k}_command_N'] for row in rows for k in range(8)) <= runner.q1_params.max_thrust
        assert policy.diagnostics(runner)['research_method'] == method
    assert len(DECAY_TIMES) == 4
