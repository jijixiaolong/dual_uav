"""Portable event compensation on the validated nominal MuJoCo control path.

The policy never changes nominal gains, event conditions, or actuator projection.
Feature schema v1 uses SI units and a persistent Q1 frame for relative quantities.
Actual motor states are available in simulation; deployment requires an actuator
state estimator or measured rotor speeds. Constraint forces are analysis-only and
are deliberately excluded from the feature vector and disturbance observer.
"""

from pathlib import Path
import sys

import mujoco
import numpy as np

CORE_BUILD_DIR = Path(__file__).resolve().parents[2] / 'core' / 'build'
if str(CORE_BUILD_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_BUILD_DIR))
import docking_core_py as dcp


EVENTS = ('CONTACT', 'DOCK', 'UNDOCK')
DECAY_TIMES = (0.08, 0.16, 0.28, 0.45)
EVENT_WINDOW_S = 1.2
MAX_MOTOR_DELTA_N = 0.25
SCHEMA_VERSION = 1
FEATURE_NAMES = (
    [f'relative_origin_error_q1_{a}_m' for a in 'xyz'] +
    [f'connector_relative_velocity_q1_{a}_mps' for a in 'xyz'] +
    [f'q2_rotation_relative_q1_{a}_rotvec_rad' for a in 'xyz'] +
    [f'q1_omega_body_{a}_radps' for a in 'xyz'] +
    [f'q2_omega_body_{a}_radps' for a in 'xyz'] +
    [f'motor{k}_actual_N' for k in range(8)] +
    ['last_total_thrust_q1_z_N'] +
    [f'last_total_torque_about_com_q1_{a}_Nm' for a in 'xyz'] +
    ['spring_force_estimate_N', 'spring_energy_estimate_J', 'motor_tau_s',
     'control_delay_s', 'bias']
)


def _vec(value):
    return np.array([value.x, value.y, value.z], dtype=float)


def _v3(value):
    return dcp.Vec3(*np.asarray(value, dtype=float))


def _coefficients8(values):
    coefficients = np.asarray(values, dtype=float)
    if coefficients.shape != (8,) or not np.isfinite(coefficients).all():
        raise ValueError('Expected exactly eight finite roll-basis coefficients')
    return coefficients.copy()


def expand_roll_coefficients(values):
    """Return [8 motors, 4 bases] in N; input order is Q1 bases then Q2 bases.

    With the configured X-quad geometry, [+,+,-,-] produces positive body-roll
    torque and zero collective, pitch, and reaction-yaw torque on each quad.
    """
    coefficients = _coefficients8(values).reshape(2, 4)
    pattern = np.array([1., 1., -1., -1.])
    return np.vstack([pattern[:, None] * coefficients[q][None, :] for q in (0, 1)])


def _rotation_vector(rotation):
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.asarray(rotation, dtype=float).ravel())
    # Shortest SO(3) logarithm, including the small-angle limit.
    if quaternion[0] < 0.:
        quaternion *= -1.
    sine = np.linalg.norm(quaternion[1:])
    if sine < 1e-12:
        return 2. * quaternion[1:]
    angle = 2. * np.arctan2(sine, np.clip(quaternion[0], 0., 1.))
    return quaternion[1:] * angle / sine


def _last_total_wrench(runner, states, rotations):
    """Last commanded total wrench about system COM, expressed in Q1 axes."""
    if runner.aggregate_control_active:
        wrench = runner.last_wrenches['aggregate']
        return np.r_[wrench.thrust, _vec(wrench.torque)]
    r1, r2 = rotations
    relative_rotation = r1.T @ r2
    w1, w2 = (runner.last_wrenches[name] for name in ('q1', 'q2'))
    force1 = np.array([0., 0., w1.thrust])
    force2 = relative_rotation @ np.array([0., 0., w2.thrust])
    positions = [_vec(state.pos) for state in states]
    m1, m2 = runner.q1_params.mass, runner.q2_params.mass
    com = (m1 * positions[0] + m2 * positions[1]) / (m1 + m2)
    moment = (_vec(w1.torque) + relative_rotation @ _vec(w2.torque) +
              np.cross(r1.T @ (positions[0] - com), force1) +
              np.cross(r1.T @ (positions[1] - com), force2))
    return np.r_[(force1 + force2)[2], moment]


def event_features(runner, event):
    """Extract and strictly validate the shared 32-component causal schema."""
    if event not in EVENTS:
        raise ValueError(f'Unsupported event {event!r}')
    world = runner.world
    states = [world.get_quad_state(q) for q in (1, 2)]
    rotations = [world.rotation(q) for q in (1, 2)]
    r1, r2 = rotations
    nominal = np.asarray(runner.config['docking']['nominal_separation_at_dock'], dtype=float)
    relative_origin = r1.T @ (_vec(states[1].pos) - _vec(states[0].pos)) - nominal
    relative_velocity = r1.T @ np.asarray(runner.latch.relative_state()['velocity'])
    options = getattr(runner, 'research_options', {})
    values = np.r_[relative_origin, relative_velocity, _rotation_vector(r1.T @ r2),
                   _vec(states[0].omega), _vec(states[1].omega),
                   np.asarray(world.applied_thrusts),
                   _last_total_wrench(runner, states, rotations),
                   float(runner.latch.preload_force), runner.latch.spring_energy(),
                   float(world.motor_tau), float(options.get('control_delay_s', 0.)), 1.]
    # The C++ boundary explicitly rejects legacy feature lengths and NaN values.
    packet = dcp.EventFeaturePacket(getattr(dcp.ResearchEvent, event), values.tolist(), SCHEMA_VERSION)
    return np.asarray(packet.values, dtype=float)


class ResearchPolicy:
    def __init__(self, method='B0', coefficients=None, selector=None):
        if method not in ('B0', 'B1', 'B2', 'B3', 'B4'):
            raise ValueError('Unknown research baseline; expected B0 through B4')
        if method == 'B4' and not callable(selector):
            raise ValueError('B4 requires a pre-trained, explicitly enabled selector')
        if method != 'B4' and selector is not None:
            raise ValueError('A conditioned selector is only allowed for B4')
        self.method, self.selector = method, selector
        supplied = dict(coefficients or {})
        unknown = set(supplied) - set(EVENTS)
        if unknown:
            raise ValueError(f'Unknown coefficient events: {sorted(unknown)}')
        self.coefficients = {event: _coefficients8(supplied.get(event, np.zeros(8)))
                             for event in EVENTS}
        # Independent objects retain overlapping CONTACT, DOCK, UNDOCK profiles.
        self.feedforwards = {event: dcp.EventMotorFeedforward() for event in EVENTS}
        for event, feedforward in self.feedforwards.items():
            feedforward.configure(EVENT_WINDOW_S, DECAY_TIMES, MAX_MOTOR_DELTA_N)
            feedforward.set_coefficients(getattr(dcp.ResearchEvent, event),
                                         expand_roll_coefficients(self.coefficients[event]).ravel().tolist())
        self.pre_event_packets = {}
        self._observers = {}
        self._compensations = {name: np.zeros(3) for name in ('q1', 'q2', 'aggregate')}
        self._mode = None
        self._moment_integrals = {name: np.zeros(3) for name in ('q1', 'q2', 'aggregate')}
        self._moment_elapsed_s = 0.
        self._last_motor_delta = np.zeros(8)
        self._last_observer_interval_s = 0.

    def reset(self, runner):
        self.pre_event_packets = {}
        for feedforward in self.feedforwards.values():
            feedforward.reset()
        self._observers = {
            'q1': dcp.AngularDisturbanceObserver(runner.q1_params.inertia, 8., .03),
            'q2': dcp.AngularDisturbanceObserver(runner.q2_params.inertia, 8., .03),
            'aggregate': dcp.AngularDisturbanceObserver(runner.alloc_conn.aggregate_inertia(), 8., .03),
        }
        self._mode = None
        self._reset_observers()
        self._last_motor_delta = np.zeros(8)

    def _reset_observers(self):
        for observer in self._observers.values():
            observer.reset()
        self._compensations = {name: np.zeros(3) for name in ('q1', 'q2', 'aggregate')}
        self._moment_integrals = {name: np.zeros(3) for name in ('q1', 'q2', 'aggregate')}
        self._moment_elapsed_s = 0.
        self._last_observer_interval_s = 0.

    def on_event(self, runner, event, t, motorcopy):
        if event not in EVENTS:
            raise ValueError(f'Unsupported event {event!r}')
        features = event_features(runner, event)
        # Oracle loads are persisted solely for attribution. They are never
        # passed to the learned selector, FF, or disturbance observer.
        weld = np.asarray(runner.latch.weld_generalized_force(), dtype=float)
        oracle_loads = {}
        for q in (1, 2):
            adr = getattr(runner.world, f'q{q}_qvel_adr')
            oracle_loads[f'q{q}_weld_force_world_N'] = weld[adr:adr + 3].tolist()
            oracle_loads[f'q{q}_weld_torque_body_Nm'] = weld[adr + 3:adr + 6].tolist()
        oracle_loads['contact_normal_force_N'] = float(runner.latch.check_contacts())
        packet = dict(time=float(t), event=event, schema_version=SCHEMA_VERSION,
                      feature_names=list(FEATURE_NAMES), values32=features.tolist(),
                      oracle_loads=oracle_loads, motor_command_N=np.asarray(motorcopy).tolist(),
                      motor_state_source='simulation filtered actual rotor thrust; deployment requires estimation')
        self.pre_event_packets[event] = packet
        if self.method in ('B1', 'B2', 'B4'):
            coefficients = (self.selector(event, features.copy()) if self.method == 'B4'
                            else self.coefficients[event])
            coefficients = _coefficients8(coefficients)
            feedforward = self.feedforwards[event]
            feedforward.set_coefficients(getattr(dcp.ResearchEvent, event),
                                         expand_roll_coefficients(coefficients).ravel().tolist())
            feedforward.trigger_event(getattr(dcp.ResearchEvent, event), float(t))
            packet['selected_coefficients8'] = coefficients.tolist()
        if event in ('DOCK', 'UNDOCK'):
            self._reset_observers()
            # The runner updates captured inertia just after the DOCK hook.
            self._mode = None

    def begin_control(self, runner, t, dt):
        if self.method != 'B3':
            return
        mode = 'aggregate' if runner.aggregate_control_active else 'independent'
        if mode != self._mode:
            self._reset_observers()
            if mode == 'aggregate':
                self._observers['aggregate'].update_inertia(runner.alloc_conn.aggregate_inertia())
            self._mode = mode
        elapsed = self._moment_elapsed_s
        interval = elapsed if elapsed > 0. else float(dt)
        if not np.isfinite(interval) or interval <= 0.:
            raise ValueError('Observer update requires a positive finite sample interval')
        names = ('aggregate',) if mode == 'aggregate' else ('q1', 'q2')
        for name in names:
            state = (runner.aggregate_state() if name == 'aggregate'
                     else runner.world.get_quad_state(int(name[-1])))
            average = self._moment_integrals[name] / elapsed if elapsed > 0. else np.zeros(3)
            wrench = self._observers[name].update(state.omega, _v3(average), interval)
            self._compensations[name] = _vec(wrench.torque)
        self._last_observer_interval_s = elapsed
        self._moment_integrals = {name: np.zeros(3) for name in ('q1', 'q2', 'aggregate')}
        self._moment_elapsed_s = 0.

    def correct_wrench(self, runner, name, wrench, t, dt):
        if name not in self._compensations:
            raise ValueError(f'Unknown wrench frame {name!r}')
        if self.method != 'B3':
            return wrench
        corrected = dcp.WrenchCommand()
        corrected.thrust = wrench.thrust
        corrected.torque = _v3(_vec(wrench.torque) + self._compensations[name])
        return corrected

    def motor_delta(self, runner, t):
        delta = np.zeros(8)
        if self.method in ('B1', 'B2', 'B4'):
            delta = sum((np.asarray(feedforward.evaluate(float(t)))
                         for feedforward in self.feedforwards.values()), np.zeros(8))
            peak = float(np.max(np.abs(delta)))
            if peak > MAX_MOTOR_DELTA_N:
                delta *= MAX_MOTOR_DELTA_N / peak
        self._last_motor_delta = delta.copy()
        return delta

    def after_step(self, runner):
        if self.method != 'B3':
            return
        actual = np.asarray(runner.world.applied_thrusts, dtype=float)
        for name, allocator, motors in (
                ('q1', runner.alloc_q1, actual[:4]),
                ('q2', runner.alloc_q2, actual[4:]),
                ('aggregate', runner.alloc_conn, actual)):
            torque = _vec(allocator.achieved_wrench(motors.tolist()).torque)
            self._moment_integrals[name] += torque * runner.world.dt
        self._moment_elapsed_s += runner.world.dt

    def diagnostics(self, runner):
        """Flat fields for the runner's physics-rate telemetry merger."""
        result = dict(research_method=self.method, research_schema_version=SCHEMA_VERSION,
                      research_motor_compensation_peak_N=float(np.max(np.abs(self._last_motor_delta))),
                      dob_observation_interval_s=self._last_observer_interval_s)
        for event, feedforward in self.feedforwards.items():
            result[f'research_{event.lower()}_ff_active'] = int(feedforward.is_active(runner.world.time))
        for name, compensation in self._compensations.items():
            for axis, value in zip('xyz', compensation):
                result[f'dob_{name}_compensation_{axis}_Nm'] = float(value)
        return result
