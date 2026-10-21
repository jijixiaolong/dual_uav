"""Cross-check the C++ constrained allocator against independent SciPy BVLS."""
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import lsq_linear

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "build"))
import docking_core_py as d


def test_asymmetric_tilted_allocator_matches_independent_bvls():
    rng = np.random.default_rng(20261021)
    worst = 0.0
    for trial in range(300):
        p1, p2 = d.QuadParams(), d.QuadParams()
        p2.mass = 0.8
        p2.max_thrust = 3.2
        p2.slew_rate_limit = 17.0
        p2.arm_length = 0.19
        p2.torque_ratio = 0.02
        p2.rotor_angles_deg = [20, 110, 200, 290]
        p2.rotor_directions = [-1, 1, -1, 1]
        allocator = d.ConnectedQuadAllocator(p1, p2, d.DockingParams())
        separation = np.array([0.08, 0.44, 0.03])
        angle = rng.uniform(-0.08, 0.08)
        allocator.update_geometry(d.Vec3(*separation), d.Quat(np.cos(angle / 2), 0, np.sin(angle / 2), 0))
        rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
        com = separation * p2.mass / (p1.mass + p2.mass)
        matrix = np.zeros((6, 8))
        for body, (params, origin, rot) in enumerate([(p1, -com, np.eye(3)), (p2, separation - com, rotation)]):
            axis = rot[:, 2]
            for i, (angle_deg, spin) in enumerate(zip(params.rotor_angles_deg, params.rotor_directions)):
                theta = np.deg2rad(angle_deg)
                position = origin + rot @ np.array([params.arm_length * np.cos(theta), params.arm_length * np.sin(theta), 0])
                matrix[:3, 4 * body + i] = axis
                matrix[3:, 4 * body + i] = np.cross(position, axis) + axis * params.torque_ratio * spin
        # Four independently controlled tasks. Lateral force is a diagnostic,
        # since captured, nonparallel rotor axes cannot make arbitrary Fx/Fy.
        scale = np.r_[1.0, np.sqrt(8 / np.maximum(np.sum(matrix[3:] ** 2, axis=1), 1e-12))]
        weighted = matrix[2:] * scale[:, None]
        physical_min = np.r_[np.full(4, p1.min_thrust), np.full(4, p2.min_thrust)]
        physical_max = np.r_[np.full(4, p1.max_thrust), np.full(4, p2.max_thrust)]
        initial = rng.uniform(physical_min, physical_max)
        allocator.set_previous_thrusts(initial.tolist())
        dt = rng.choice([0.004, 0.006, 1.0])
        slew = np.r_[np.full(4, p1.slew_rate_limit), np.full(4, p2.slew_rate_limit)]
        lower = np.maximum(physical_min, initial - slew * dt)
        upper = np.minimum(physical_max, initial + slew * dt)
        desired = np.r_[0, 0, rng.uniform(0, 45), rng.uniform(-4, 4, 3)]
        command = d.WrenchCommand()
        command.thrust, command.torque = desired[2], d.Vec3(*desired[3:])
        actual = np.array(allocator.allocate(command, dt))
        oracle = lsq_linear(weighted, desired[2:] * scale, bounds=(lower, upper), method="bvls", tol=1e-12, max_iter=300)
        assert oracle.success
        excess = np.linalg.norm(weighted @ actual - desired[2:] * scale) ** 2 - np.linalg.norm(weighted @ oracle.x - desired[2:] * scale) ** 2
        worst = max(worst, excess)
        assert np.all(actual >= lower - 1e-12) and np.all(actual <= upper + 1e-12)
        assert excess < 1e-7, (trial, excess)
    print(f"300 independent BVLS comparisons; maximum excess squared residual: {worst:.3g}")
