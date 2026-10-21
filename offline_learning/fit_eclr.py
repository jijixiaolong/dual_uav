"""ECLR (Event-Conditioned Lifted Residual) identification and parameter export for dual quadrotor docking."""
import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / 'core' / 'build'))

from mujoco_sim.python.sim_runner import SimulationRunner
import docking_core_py as dcp


def sample_pre_event_conditions(n_samples: int, seed: int = 42, test: bool = False):
    """Sample realistic approach offsets and speed variations."""
    rng = np.random.default_rng(seed)
    conditions = []
    for _ in range(n_samples):
        if not test:
            # Training domain: slight lateral offset, velocity perturbation
            offset = rng.uniform(-0.015, 0.015)
            q1_init_y = rng.uniform(-0.62, -0.58)
            q2_init_y = rng.uniform(0.58, 0.62)
            c_probe = rng.uniform(-0.6, 0.6, size=4)
        else:
            # Held-out testing domain: wider offset to test generalization
            offset = rng.uniform(-0.025, 0.025)
            q1_init_y = rng.uniform(-0.65, -0.55)
            q2_init_y = rng.uniform(0.55, 0.65)
            c_probe = np.zeros(4)

        conditions.append({
            'approach_offset': float(offset),
            'q1_init_y': float(q1_init_y),
            'q2_init_y': float(q2_init_y),
            'c_probe': c_probe
        })
    return conditions


def collect_eclr_dataset(runner: SimulationRunner, conditions: list, mode: str = 'dock'):
    """Run simulation trials and collect pre-event features xi and lifted responses y."""
    X_list = []
    Y_list = []

    for idx, cond in enumerate(conditions):
        runner.configure_compensator(dcp.CompensatorType.FIXED_FF, coeff=cond['c_probe'])
        logs = runner.run_scenario(
            duration=6.5,
            dock_cmd_time=1.0,
            undock_cmd_time=4.5,
            approach_offset=cond['approach_offset'],
            q1_init_y=cond['q1_init_y'],
            q2_init_y=cond['q2_init_y']
        )

        # Locate event time
        event_logs = []
        if mode == 'dock':
            # Extract logs for 1.2s after entering CONNECTED
            conn_steps = [l for l in logs if l['mode_name'] == 'CONNECTED']
            if not conn_steps:
                continue
            t0 = conn_steps[0]['time']
            event_logs = [l for l in conn_steps if 0.0 <= (l['time'] - t0) <= 1.2]
        else: # undock
            sep_steps = [l for l in logs if l['mode_name'] in ('SEPARATING', 'RECOVERY')]
            if not sep_steps:
                continue
            t0 = sep_steps[0]['time']
            event_logs = [l for l in sep_steps if 0.0 <= (l['time'] - t0) <= 1.2]

        if len(event_logs) < 20:
            continue

        # Sample 15 time points along the 1.2s post-event horizon
        sample_indices = np.linspace(0, len(event_logs) - 1, 15, dtype=int)
        pitch_errors = [event_logs[i]['q1_pitch_deg'] for i in sample_indices]
        rate_errors = [event_logs[i]['q1_pitch_rate'] for i in sample_indices]

        # Lifted output y: concatenated attitude errors and rates
        y_lifted = np.array(pitch_errors + rate_errors, dtype=np.float64)

        # Pre-event feature phi(xi)
        feat = np.array([
            cond['approach_offset'],
            cond['q1_init_y'] - (-0.6),
            cond['q2_init_y'] - 0.6,
            1.0 # bias term
        ], dtype=np.float64)

        # Full regression input: [phi(xi), c]
        x_full = np.concatenate([feat, cond['c_probe']])
        X_list.append(x_full)
        Y_list.append(y_lifted)

    return np.array(X_list), np.array(Y_list)


def fit_eclr_matrices(X: np.ndarray, Y: np.ndarray, reg_lambda: float = 1e-4):
    """Solve ridge regression for A and G matrices: Y = X * W + eps."""
    n_features = 4 # dim of phi(xi)
    n_controls = 4 # dim of basis coefficients c

    reg = reg_lambda * np.eye(X.shape[1])
    # W: (8 x 30)
    W = np.linalg.solve(X.T @ X + reg, X.T @ Y)

    A = W[:n_features, :].T # (30 x 4)
    G = W[n_features:, :].T # (30 x 4)

    return A, G


def compute_fixed_feedforward(A: np.ndarray, G: np.ndarray, reg_lambda: float = 1e-3, box_limit: float = 1.0):
    """Compute optimal fixed feedforward for the nominal condition."""
    nominal_phi = np.array([0.0, 0.0, 0.0, 1.0])
    y0 = A @ nominal_phi
    c_fixed = -np.linalg.solve(G.T @ G + reg_lambda * np.eye(G.shape[1]), G.T @ y0)
    return np.clip(c_fixed, -box_limit, box_limit)

