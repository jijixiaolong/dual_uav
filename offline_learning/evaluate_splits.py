"""Legacy call interface, using the formal real-event metric implementation.

The legacy learner remains gated by SimulationRunner.  Current research uses
the repaired research entry point and does not train from this old pipeline.
"""
import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / 'core' / 'build'))

from mujoco_sim.python.sim_runner import SimulationRunner
import docking_core_py as dcp
from analysis_tools.metrics import evaluate_events, event_window


def evaluate_trial(runner: SimulationRunner,
                   method: str,
                   condition: dict,
                   c_fixed: np.ndarray = None,
                   A: np.ndarray = None,
                   G: np.ndarray = None) -> dict:
    """Retain the old input interface while returning auditable event metrics.

    Historical keys remain available. ``peak_pitch_deg`` and ``window_pitches``
    now mean signed pitch *tracking error*, rather than absolute pitch; settling
    uses the joint SO(3)/position/angular-rate tolerance.  Unrecovered or absent
    events produce None, and failed trials retain their classifications.
    """
    if method == 'baseline':
        runner.configure_compensator(dcp.CompensatorType.NONE)
    elif method in ('fixed_ff', 'ilc'):
        coeff = c_fixed if c_fixed is not None else np.zeros(4)
        runner.configure_compensator(dcp.CompensatorType.FIXED_FF, coeff=coeff)
    elif method == 'eclr':
        # Provide A and G to C++ core
        A_list = [list(row) for row in A] if A is not None else []
        G_list = [list(row) for row in G] if G is not None else []
        runner.configure_compensator(dcp.CompensatorType.ECLR, A=A_list, G=G_list)
    else:
        raise ValueError(f'Unknown legacy controller method: {method}')

    simulation_exception = None
    try:
        logs = runner.run_scenario(
            duration=6.5, dock_cmd_time=1.0, undock_cmd_time=4.5,
            approach_offset=condition['approach_offset'],
            q1_init_y=condition['q1_init_y'], q2_init_y=condition['q2_init_y'])
    except (RuntimeError, FloatingPointError) as exc:
        simulation_exception = f'{type(exc).__name__}: {exc}'
        logs = list(getattr(runner, 'telemetry', []))
    events = list(getattr(runner, 'events', []))
    if simulation_exception and hasattr(runner, 'latch'):
        events = list(runner.latch.events)
    motor_config = getattr(runner, 'config', {}).get('quadrotor', {}).get('motor', {})
    evaluated = evaluate_events(logs, events, {
        'motor_min_N': motor_config.get('min_thrust', .05),
        'motor_max_N': motor_config.get('max_thrust', 5.5)})
    if simulation_exception:
        evaluated['outcome'].update(category='failure', reason='simulation_exception',
                                    failed=True, timed_out=False)
    dock = next(record for record in evaluated['events'] if record['event'] == 'DOCK')
    metrics = dock['metrics']
    result = dict(success=evaluated['outcome']['category'] == 'success', method=method,
                  peak_pitch_deg=None, peak_rate_deg_s=None, peak_attitude_error_deg=None,
                  rms_pitch_deg=None, settling_time_s=None, right_censored=None,
                  window_times=[], window_pitches=[], event_metrics=evaluated,
                  outcome=evaluated['outcome'], simulation_exception=simulation_exception)
    if metrics is None:
        return result
    window, _ = event_window(logs, dock['event_time_s'], 1.2)
    times = np.asarray([float(r['time'])-dock['event_time_s'] for r in window])
    pitches = np.asarray([np.nan if r.get('q1_pitch_error_deg') in (None, '')
                          else float(r['q1_pitch_error_deg']) for r in window])
    rms_pitch = None
    if len(times) > 1 and times[-1] > times[0] and np.isfinite(pitches).all():
        a, b = pitches[:-1], pitches[1:]
        rms_pitch = float(np.sqrt(np.sum(np.diff(times)*(a*a+a*b+b*b)/3)/(times[-1]-times[0])))
    result.update(peak_pitch_deg=metrics['q1']['peak_pitch_error_deg'],
                  peak_rate_deg_s=metrics['q1']['peak_rate_deg_s'],
                  peak_attitude_error_deg=metrics['combined']['peak_attitude_error_deg'],
                  rms_pitch_deg=rms_pitch,
                  settling_time_s=metrics['combined']['settling_time_s'],
                  right_censored=metrics['combined']['right_censored'],
                  window_times=times.tolist(),
                  window_pitches=[float(p) if np.isfinite(p) else None for p in pitches])
    return result

