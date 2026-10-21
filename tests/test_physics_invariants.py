"""Physical invariants across collision, capture, release and actuator dynamics.

These checks deliberately use engine state and independent momentum/energy sums,
so a plausible looking trajectory cannot hide an incorrect latch implementation.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "core" / "build"))

from mujoco_sim.python.mujoco_world import MuJoCoWorld
from mujoco_sim.python.latch_manager import LatchManager

XML_PATH = PROJECT_ROOT / "mujoco_sim" / "assets" / "dual_quads.xml"


def _world(dt=0.001):
    world = MuJoCoWorld(str(XML_PATH), dt=dt, motor_tau=0.0)
    world.model.opt.gravity[:] = 0.0
    world.data.ctrl[:] = 0.0
    world.applied_thrusts[:] = 0.0
    world.target_thrusts[:] = 0.0
    return world


def _totals(world):
    """World-frame momentum and kinetic energy from the two native rigid bodies."""
    momentum = np.zeros(3)
    angular = np.zeros(3)
    kinetic = 0.0
    for body_id, qpos, qvel in (
        (world.q1_body_id, world.q1_qpos_adr, world.q1_qvel_adr),
        (world.q2_body_id, world.q2_qpos_adr, world.q2_qvel_adr),
    ):
        mass = float(world.model.body_mass[body_id])
        velocity = world.data.qvel[qvel:qvel + 3]
        omega_body = world.data.qvel[qvel + 3:qvel + 6]
        rotation = world.data.xmat[body_id].reshape(3, 3)
        inertia_diag = world.model.body_inertia[body_id]
        linear = mass * velocity
        momentum += linear
        angular += np.cross(world.data.qpos[qpos:qpos + 3], linear)
        angular += rotation @ (inertia_diag * omega_body)
        kinetic += 0.5 * mass * float(velocity @ velocity)
        kinetic += 0.5 * float(omega_body @ (inertia_diag * omega_body))
    return momentum, angular, kinetic


def test_unforced_connector_collision_conserves_linear_momentum_and_dissipates_energy():
    world = _world()
    world.reset(q1_init_pos=(-0.24, 2.0), q2_init_pos=(0.24, 2.0))
    world.data.ctrl[:] = 0.0
    world.data.qvel[world.q1_qvel_adr:world.q1_qvel_adr + 3] = [0.07, 0.25, 0.0]
    world.data.qvel[world.q2_qvel_adr:world.q2_qvel_adr + 3] = [0.07, -0.15, 0.0]
    mujoco.mj_forward(world.model, world.data)
    momentum0, _, energy0 = _totals(world)
    latch = LatchManager(world)
    peak_contact_force = 0.0
    peak_momentum_error = 0.0
    for _ in range(600):
        mujoco.mj_step(world.model, world.data)
        mujoco.mj_forward(world.model, world.data)
        peak_contact_force = max(peak_contact_force, latch.check_contacts())
        momentum, _, _ = _totals(world)
        peak_momentum_error = max(peak_momentum_error, np.linalg.norm(momentum - momentum0))
    _, _, energy_final = _totals(world)
    assert peak_contact_force > 0.01, "The test must include an actual connector collision."
    assert peak_momentum_error < 1e-8
    assert energy_final < energy0 - 1e-5, "Passive contact must dissipate approach energy."
    assert not world.is_weld_active()


def test_xml_motors_really_turn_off_without_residual_idle_thrust():
    world = _world()
    world.reset(q1_init_pos=(-0.6, 2.0), q2_init_pos=(0.6, 2.0))
    world.data.ctrl[:] = 0.0
    world.data.qvel[:] = 0.0
    for _ in range(100):
        mujoco.mj_step(world.model, world.data)
    np.testing.assert_allclose(world.data.qvel, 0.0, atol=1e-12)
    np.testing.assert_allclose(world.data.actuator_force, 0.0, atol=1e-12)


def test_final_motor_path_obeys_amplitude_and_rate_limits_after_arbitrary_input():
    world = MuJoCoWorld(str(XML_PATH), dt=0.002, motor_tau=0.025)
    previous_target = world.target_thrusts.copy()
    previous_actual = world.applied_thrusts.copy()
    # Alternating inputs deliberately exceed both ends of the physical range.
    for step in range(250):
        command = np.full(8, 20.0 if (step // 15) % 2 else -20.0)
        world.step(command)
        for values, previous in (
            (world.target_thrusts, previous_target),
            (world.applied_thrusts, previous_actual),
        ):
            assert np.all(values >= -1e-12)
            assert np.all(values <= 5.5 + 1e-12)
            assert np.max(np.abs(values - previous)) <= 45.0 * world.dt + 1e-12
        np.testing.assert_allclose(world.data.ctrl, world.applied_thrusts)
        previous_target = world.target_thrusts.copy()
        previous_actual = world.applied_thrusts.copy()


@pytest.mark.parametrize("command", [np.zeros(7), np.full(8, np.nan), np.full(8, np.inf)])
def test_nonfinite_or_wrong_size_motor_commands_do_not_advance_simulation(command):
    world = _world()
    before = world.data.qpos.copy()
    with pytest.raises(ValueError):
        world.step(command)
    assert world.time == 0.0
    np.testing.assert_array_equal(world.data.qpos, before)


def test_reported_momentum_and_energy_match_independent_rigid_body_sum():
    world = _world()
    world.data.qpos[world.q1_qpos_adr + 3:world.q1_qpos_adr + 7] = [np.cos(0.2), 0, 0, np.sin(0.2)]
    world.data.qpos[world.q2_qpos_adr + 3:world.q2_qpos_adr + 7] = [np.cos(0.3), np.sin(0.3), 0, 0]
    world.data.qvel[:] = [0.2, -0.1, 0.3, 0.4, 0.1, -0.2, -0.1, 0.3, 0.2, -0.2, 0.3, 0.1]
    mujoco.mj_forward(world.model, world.data)
    momentum, angular, kinetic = _totals(world)
    reported = world.system_quantities()
    np.testing.assert_allclose(reported['linear_momentum'], momentum, atol=1e-12)
    np.testing.assert_allclose(reported['angular_momentum'], angular, atol=1e-12)
    assert reported['kinetic_energy_J'] == pytest.approx(kinetic, abs=1e-12)


def test_capture_requires_real_contact_even_inside_position_tolerance():
    world = _world()
    world.reset(q1_init_pos=(-0.225, 2.0), q2_init_pos=(0.225, 2.0))
    latch = LatchManager(world)
    world.forward()
    assert np.linalg.norm(latch.relative_state()['position']) < latch.params['position_tolerance_m']
    assert not latch.contact_details()
    assert not latch.capture_eligible()
    assert not latch.engage_weld()
    assert not world.is_weld_active()


def test_contact_capture_checks_speed_and_preserves_state_at_switch():
    world = _world()
    world.reset(q1_init_pos=(-0.2199, 2.0), q2_init_pos=(0.2199, 2.0))
    latch = LatchManager(world)
    world.forward()
    assert latch.contact_details(), "Slightly compressed connector faces must touch."
    world.data.qvel[world.q2_qvel_adr + 1] = 0.3
    world.forward()
    assert not latch.capture_eligible()
    assert not latch.engage_weld()
    world.data.qvel[:] = 0.0
    world.forward()
    assert latch.capture_eligible()
    qpos, qvel = world.data.qpos.copy(), world.data.qvel.copy()
    assert latch.engage_weld()
    np.testing.assert_array_equal(world.data.qpos, qpos)
    np.testing.assert_array_equal(world.data.qvel, qvel)
    assert world.is_weld_active()


def test_weld_reference_uses_actual_relative_pose_without_initial_constraint_error():
    world = _world()
    world.reset(q1_init_pos=(-0.23, 2.0), q2_init_pos=(0.23, 2.006))
    # Deliberately differ from the XML's nominal 0.44 m / identity relative pose.
    world.data.qpos[world.q2_qpos_adr] = 0.004
    world.data.qpos[world.q2_qpos_adr + 3:world.q2_qpos_adr + 7] = [np.cos(0.01), 0, 0, np.sin(0.01)]
    world.forward()
    latch = LatchManager(world)
    qpos, qvel = world.data.qpos.copy(), world.data.qvel.copy()
    assert latch.initialize_connected()
    np.testing.assert_array_equal(world.data.qpos, qpos)
    np.testing.assert_array_equal(world.data.qvel, qvel)
    equality_rows = world.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    np.testing.assert_allclose(world.data.efc_pos[equality_rows], 0.0, atol=1e-12)
    np.testing.assert_allclose(latch.weld_generalized_force(), 0.0, atol=1e-9)


def test_zero_preload_release_preserves_all_velocities_momentum_and_energy():
    world = _world()
    world.reset(q1_init_pos=(-0.2225, 2.0), q2_init_pos=(0.2225, 2.0))
    latch = LatchManager(world, zero_preload=True)
    # A translating and rotating pair; a finite gap prevents incidental contact.
    world.data.qvel[world.q1_qvel_adr:world.q1_qvel_adr + 6] = [0.1267, 0.04, -0.02, 0.0, 0.0, 0.12]
    world.data.qvel[world.q2_qvel_adr:world.q2_qvel_adr + 6] = [0.0733, 0.04, -0.02, 0.0, 0.0, 0.12]
    world.forward()
    latch.initialize_connected()
    qpos, qvel = world.data.qpos.copy(), world.data.qvel.copy()
    momentum0, angular0, energy0 = _totals(world)
    assert latch.spring_energy() == 0.0
    assert latch.disengage_weld()
    np.testing.assert_array_equal(world.data.qpos, qpos)
    np.testing.assert_array_equal(world.data.qvel, qvel)
    np.testing.assert_allclose(latch.spring_force_generalized(), 0.0, atol=0.0)
    for _ in range(100):
        world.step(np.zeros(8))
    momentum, angular, energy = _totals(world)
    np.testing.assert_allclose(world.data.qvel, qvel, atol=1e-10)
    np.testing.assert_allclose(momentum, momentum0, atol=1e-10)
    np.testing.assert_allclose(angular, angular0, atol=1e-8)
    assert energy == pytest.approx(energy0, abs=1e-10)


def test_preloaded_release_has_finite_energy_and_balanced_internal_force():
    world = _world(dt=0.00025)
    world.reset(q1_init_pos=(-0.22, 2.0), q2_init_pos=(0.22, 2.0))
    latch = LatchManager(world, zero_preload=False)
    assert latch.initialize_connected()
    initial_storage = 0.5 * latch.spring_stiffness * latch.preload_deflection ** 2
    assert initial_storage == pytest.approx(0.00192)
    assert latch.spring_energy() == pytest.approx(initial_storage)
    assert latch.events[-1]['spring_charging_work_J'] == pytest.approx(initial_storage)
    force = latch.spring_force_generalized()
    force1 = force[world.q1_qvel_adr:world.q1_qvel_adr + 3]
    force2 = force[world.q2_qvel_adr:world.q2_qvel_adr + 3]
    np.testing.assert_allclose(force1 + force2, 0.0, atol=1e-12)
    assert np.linalg.norm(force1) == pytest.approx(0.48)
    # Let the finite preload actually load the equality constraint before release.
    for _ in range(200):
        world.step(np.zeros(8))
    assert np.linalg.norm(latch.weld_generalized_force()) > 0.1
    qvel = world.data.qvel.copy()
    momentum0, angular0, kinetic0 = _totals(world)
    budget = kinetic0 + latch.spring_energy()
    assert latch.disengage_weld()
    np.testing.assert_array_equal(world.data.qvel, qvel)
    max_energy = 0.0
    for _ in range(2400):
        world.step(np.zeros(8))
        momentum, angular, kinetic = _totals(world)
        np.testing.assert_allclose(momentum, momentum0, atol=1e-9)
        np.testing.assert_allclose(angular, angular0, atol=1e-8)
        max_energy = max(max_energy, kinetic + latch.spring_energy())
    assert kinetic > 1e-5, "Stored preload energy should create a measurable separation response."
    assert max_energy <= budget + 1e-5, "Release cannot inject unaccounted mechanical energy."
    assert latch.spring_energy() < 1e-10
    np.testing.assert_allclose(latch.spring_force_generalized(), 0.0, atol=1e-10)
    np.testing.assert_allclose(world.data.qfrc_applied, 0.0, atol=1e-10)


def test_preload_is_an_internal_wrench_with_transverse_connector_misalignment():
    world = _world()
    world.reset(q1_init_pos=(-0.22, 2.0), q2_init_pos=(0.22, 2.003))
    world.data.qpos[world.q2_qpos_adr] = 0.004
    world.forward()
    latch = LatchManager(world, zero_preload=False)
    latch.initialize_connected()
    generalized = latch.spring_force_generalized()
    net_force = np.zeros(3)
    net_moment_world = np.zeros(3)
    for index in (1, 2):
        a = getattr(world, f'q{index}_qpos_adr')
        v = getattr(world, f'q{index}_qvel_adr')
        force = generalized[v:v + 3]
        torque_world = world.rotation(index) @ generalized[v + 3:v + 6]
        net_force += force
        net_moment_world += np.cross(world.data.qpos[a:a + 3], force) + torque_world
    np.testing.assert_allclose(net_force, 0.0, atol=1e-12)
    np.testing.assert_allclose(net_moment_world, 0.0, atol=1e-12)
