"""Surrogate benchmark ONLY: two different planar attitude-transition event models.

NOT a six-DoF docking simulator, not a contact physics engine and not evidence
of a real quadrotor controller's performance. It demonstrates the lifted
identification + constrained event-conditioned correction pipeline.
"""
from dataclasses import dataclass
from typing import Optional
import numpy as np

DT = 0.01
HORIZON = 1.5
TIME = np.arange(0.0, HORIZON + DT / 2, DT)
SAMPLE_INDICES = np.arange(2, len(TIME), 6)

@dataclass(frozen=True)
class Experiment:
    mode: str
    feature: np.ndarray
    coeff: np.ndarray
    response: np.ndarray


def sample_feature(rng: np.random.Generator, mode: str, test=False):
    """Physical-looking event descriptors; illustrative ranges, NOT safe hardware limits."""
    if mode == 'dock':
        v = rng.uniform(0.04, 0.17) if not test else rng.uniform(0.02, 0.19)
        theta0 = rng.uniform(-0.06, 0.06) if not test else rng.uniform(-0.075, 0.075)
        mismatch = rng.uniform(-0.14, 0.14) if not test else rng.uniform(-0.17, 0.17)
        return np.array([v, theta0, mismatch])
    if mode == 'undock':
        preload = rng.uniform(-0.12, 0.12) if not test else rng.uniform(-0.15, 0.15)
        theta0 = rng.uniform(-0.05, 0.05) if not test else rng.uniform(-0.07, 0.07)
        mismatch = rng.uniform(-0.12, 0.12) if not test else rng.uniform(-0.16, 0.16)
        return np.array([preload, theta0, mismatch])
    raise ValueError(mode)


def phi(feature: np.ndarray, mode: str):
    a, pitch, mismatch = feature
    if mode == 'dock':
        return np.array([1., (a - 0.1) / .1, pitch / .1, mismatch / .2])
    return np.array([1., a / .15, pitch / .1, mismatch / .2])


def run(mode: str, feature: np.ndarray, coeff: Optional[np.ndarray] = None, variation=0.0):
    """Return time, pitch angle [rad], pitch rate [rad/s], correction [rad/s^2].

    Both events use a closed-loop linearized rotational system with explicitly
    modeled event perturbations. Free and assembled structures have different
    inertia; correction is expressed as angular acceleration for comparability.
    'variation' perturbs event amplitude to test robustness; not an empirical model.
    """
    c = np.zeros(2) if coeff is None else np.asarray(coeff, dtype=float)
    if c.shape != (2,):
        raise ValueError('correction coefficient must have shape (2,)')
    if mode == 'dock':
        speed, pitch0, mismatch = feature
        pitch_rate0 = -1.5 * speed + 0.30 * mismatch
        amp = 6.5 * ((speed - 0.08) / 0.1) + 2.0 * pitch0 / .1 + 2.5 * mismatch / .2
        ktheta, kw = 18.0, 7.0
        tau = 0.13
    elif mode == 'undock':
        preload, pitch0, mismatch = feature
        pitch_rate0 = 0.8 * preload
        amp = 5.5 * preload / .15 - 1.8 * pitch0 / .1 + 2.4 * mismatch / .2
        ktheta, kw = 28.0, 9.0
        tau = 0.10
    else:
        raise ValueError(mode)
    amp += variation
    y = np.empty((len(TIME), 2))
    u = np.empty(len(TIME))
    y[0] = [pitch0, pitch_rate0]
    for i, t in enumerate(TIME):
        b0 = np.exp(-t / 0.20)
        b1 = (t / 0.16) * np.exp(-t / 0.16)
        # A rate-limited implementation would constrain torque time-derivatives
        # at the allocator; this analytic basis is smooth after event triggering.
        u[i] = np.clip(c[0] * b0 + c[1] * b1, -8., 8.)
        if i == len(TIME)-1:
            break
        pulse = amp * np.exp(-t / tau)
        theta, omega = y[i]
        # Semi-implicit Euler for this *surrogate*, not the intended 6-DoF plant.
        omega_next = omega + DT * (-ktheta*theta - kw*omega + pulse + u[i])
        theta_next = theta + DT * omega_next
        y[i+1] = [theta_next, omega_next]
    return TIME, y[:, 0], y[:, 1], u


def lifted_response(mode, feature, coeff=None, variation=0.0):
    _, theta, omega, _ = run(mode, feature, coeff, variation)
    # Rich output: pitch and pitch rate over the event horizon.
    return np.concatenate([theta[SAMPLE_INDICES], 0.20*omega[SAMPLE_INDICES]])


def fit_lifted(mode, n_train: int, seed: int):
    rng = np.random.default_rng(seed)
    X, Y = [], []
    for _ in range(n_train):
        f = sample_feature(rng, mode)
        # Random safe-in-surrogate probing; not a safe excitation on real aircraft.
        c = rng.uniform(-3.0, 3.0, size=2)
        X.append(np.r_[phi(f, mode), c])
        Y.append(lifted_response(mode, f, c))
    X, Y = np.asarray(X), np.asarray(Y)
    # Ridge regularization helps with small-sample conditioning.
    reg = 1e-5 * np.eye(X.shape[1])
    W = np.linalg.solve(X.T @ X + reg, X.T @ Y)
    A, G = W[:4, :].T, W[4:, :].T
    return A, G


def choose_c(mode, feature, A, G, limit=6., reg=0.004):
    y0 = A @ phi(feature, mode)
    c = -np.linalg.solve(G.T @ G + reg * np.eye(2), G.T @ y0)
    return np.clip(c, -limit, limit)


def score(y):
    # normalized lifted squared error; not an established physical metric.
    return float(np.mean(y*y))


def evaluate(mode, n_train=40, seed=4, n_test=100):
    A, G = fit_lifted(mode, n_train, seed)
    rng = np.random.default_rng(seed+10000)
    # fixed waveform is fitted for mean training condition; no personalization.
    nominal = np.array([0.10, 0., 0.]) if mode == 'dock' else np.array([0., 0., 0.])
    c_fixed = choose_c(mode, nominal, A, G)
    outcomes = []
    for _ in range(n_test):
        f = sample_feature(rng, mode, test=True)
        variation = rng.normal(scale=0.20)
        c_cond = choose_c(mode, f, A, G)
        results = {}
        for name, coeff in [('baseline', np.zeros(2)), ('fixed_ff', c_fixed), ('event_learning', c_cond)]:
            _, theta, omega, u = run(mode, f, coeff, variation)
            results[name] = {'score':score(lifted_response(mode, f, coeff, variation)),
                             'peak_theta_deg':float(np.max(np.abs(theta))*180/np.pi),
                             'peak_rate_deg_s':float(np.max(np.abs(omega))*180/np.pi),
                             'input_energy':float(np.trapz(u*u, TIME))}
        outcomes.append(results)
    return outcomes, (A, G)
