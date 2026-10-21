import sys
from pathlib import Path
import pytest
import numpy as np

# Ensure path to core/build and project root
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / 'core' / 'build'))

from mujoco_sim.python.mujoco_world import MuJoCoWorld
import docking_core_py as dcp


def test_freefall_gravity():
    """Verify free-fall acceleration matches gravity exactly with motors off."""
    xml_path = str(PROJECT_ROOT / 'mujoco_sim' / 'assets' / 'dual_quads.xml')
    world = MuJoCoWorld(xml_path, dt=0.002)
    world.reset(q1_init_pos=(-0.6, 2.0), q2_init_pos=(0.6, 2.0))

    # Disable all motors and motor lag for pure freefall verification
    world.applied_thrusts[:] = 0.0
    world.target_thrusts[:] = 0.0
    world.motor_tau = 0.0
    zero_cmd = np.zeros(8)
    for _ in range(100): # 0.2s of free fall
        world.step(zero_cmd)

    s1 = world.get_quad_state(1)
    # v = g * t => v_z ~ -9.80665 * 0.2 = -1.961 m/s
    expected_vz = -9.80665 * world.time
    assert abs(s1.vel.z - expected_vz) < 1e-10


def test_single_and_dual_hover_equilibrium():
    """Verify steady-state hover stability and motor thrust matches weight."""
    xml_path = str(PROJECT_ROOT / 'mujoco_sim' / 'assets' / 'dual_quads.xml')
    world = MuJoCoWorld(xml_path, dt=0.002)
    world.reset(q1_init_pos=(-0.6, 1.0), q2_init_pos=(0.6, 1.0))

    qp = dcp.QuadParams()
    gains = dcp.SE3Gains()
    ctrl1 = dcp.GeometricSE3Controller(qp.mass, dcp.Mat3.Identity(), gains)
    ctrl2 = dcp.GeometricSE3Controller(qp.mass, dcp.Mat3.Identity(), gains)
    alloc1 = dcp.SingleQuadAllocator(qp)
    alloc2 = dcp.SingleQuadAllocator(qp)

    sp1 = dcp.Setpoint()
    sp1.pos = dcp.Vec3(0.0, -0.6, 1.0)
    sp2 = dcp.Setpoint()
    sp2.pos = dcp.Vec3(0.0, 0.6, 1.0)

    # Hover for 1.5s
    for step in range(750):
        t = world.time
        if step % 2 == 0:
            s1 = world.get_quad_state(1)
            s2 = world.get_quad_state(2)
            w1 = ctrl1.compute(s1, sp1, 0.004)
            w2 = ctrl2.compute(s2, sp2, 0.004)
            u1 = alloc1.allocate(w1, 0.004)
            u2 = alloc2.allocate(w2, 0.004)
            cmd8 = list(u1) + list(u2)
        world.step(cmd8)

    s1 = world.get_quad_state(1)
    s2 = world.get_quad_state(2)

    # Positions stay within 3 cm of target
    assert abs(s1.pos.z - 1.0) < 0.03
    assert abs(s1.pos.y - (-0.6)) < 0.03
    assert abs(s2.pos.z - 1.0) < 0.03
    assert abs(s2.pos.y - 0.6) < 0.03

    # Rotor thrusts hover around 2.45 N each
    for t_val in cmd8:
        assert 2.2 < t_val < 2.7
