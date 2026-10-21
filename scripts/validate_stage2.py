#!/usr/bin/env python3
"""Rebuild, test and record P0/P1 only. Exit nonzero if any acceptance gate fails."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
from importlib.metadata import version
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def run_command(command,path):
    with path.open('w') as log:
        result = subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    print(f'{"PASS" if result.returncode == 0 else "FAIL"}: {" ".join(map(str,command))}',flush=True)
    if result.returncode:
        raise RuntimeError(f'Gate failed; see {path}')


def source_hashes():
    files = [ROOT/name for name in ('run_experiments.py','core/CMakeLists.txt','requirements.txt','requirements-validation.lock.txt')]
    for directory in ('core/src','core/include','core/tests','mujoco_sim','configs','tests','scripts','analysis_tools'):
        files += [p for p in (ROOT/directory).rglob('*') if p.suffix in ('.py','.cpp','.hpp','.xml','.yaml')]
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}


def plot_case(path,rows,events):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    t = np.array([r['time'] for r in rows])
    fig,axes = plt.subplots(4,2,figsize=(13,12),sharex=True)
    for i,color in ((1,'tab:blue'),(2,'tab:red')):
        for j,key,label in ((0,'roll_error_deg','Roll tracking error (deg)'),(1,'pitch_error_deg','Pitch tracking error (deg)')):
            axes[0,j].plot(t,[r[f'q{i}_{key}'] for r in rows],label=f'Q{i}',color=color)
            axes[0,j].set_ylabel(label)
        axes[1,0].plot(t,[r[f'q{i}_omega_body_x'] for r in rows],label=f'Q{i} body wx',color=color)
        axes[2,0].plot(t,[r[f'q{i}_pos_error_m'] for r in rows],label=f'Q{i}',color=color)
    axes[1,0].set_ylabel('Angular velocity (rad/s)')
    axes[1,1].plot(t,[r['contact_normal_force_N'] for r in rows],label='Contact normal sum')
    axes[1,1].plot(t,[np.linalg.norm([r[f'weld_q1_force_world_N_{a}'] for a in 'xyz']) for r in rows],label='Weld force on Q1')
    axes[1,1].set_ylabel('Force (N)')
    axes[2,0].set_ylabel('Position tracking error (m)')
    for k in range(8):
        axes[2,1].plot(t,[r[f'motor{k}_actual_N'] for r in rows],alpha=.7,linewidth=.65)
    axes[2,1].set_ylabel('Eight actual motor thrusts (N)')
    axes[3,0].plot(t,[r['spring_force_N'] for r in rows],label='Spring force (N)')
    axes[3,0].plot(t,[r['spring_energy_J']*1000 for r in rows],label='Spring energy (mJ)')
    axes[3,0].set_ylabel('Release spring')
    axes[3,1].step(t,[r['weld_active'] for r in rows],label='Weld active')
    axes[3,1].plot(t,[r['rel_dist'] for r in rows],label='Connector error (m)')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        for event in events:
            if event['event'] != 'INITIAL_CONNECTED':
                ax.axvline(event['time'],color='k',linestyle=':',alpha=.5)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=7,loc='best')
    axes[3,0].set_xlabel('Simulation time (s)')
    axes[3,1].set_xlabel('Simulation time (s)')
    fig.suptitle(f'{path.name} | real MuJoCo, no learning')
    fig.tight_layout()
    fig.savefig(path/'diagnostics.png',dpi=150)
    for event in events:
        if event['event'] == 'INITIAL_CONNECTED':
            continue
        for ax in axes.flat:
            ax.set_xlim(event['time']-.2,event['time']+1.2)
        fig.savefig(path/f"{event['event'].lower()}_window.png",dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111,projection='3d')
    for i in (1,2):
        ax.plot([r[f'q{i}_x'] for r in rows],[r[f'q{i}_y'] for r in rows],[r[f'q{i}_z'] for r in rows],label=f'Q{i}')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.legend(); fig.tight_layout(); fig.savefig(path/'trajectory_3d.png',dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results'/'physics_validation')
    parser.add_argument('--skip-build',action='store_true')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    manifest = dict(stage='P0_P1',learning_enabled=False,python=sys.version,platform=platform.platform(),
                    deterministic=True,random_disturbances=False,source_sha256=source_hashes(),
                    packages={p:version(p) for p in ('mujoco','numpy','scipy','matplotlib','PyYAML','pybind11','pytest','Pillow')})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    if not args.skip_build:
        run_command(['cmake','-S','core','-B','core/build','-DCMAKE_BUILD_TYPE=Release'],output/'build_configure.log')
        run_command(['cmake','--build','core/build','-j','4'],output/'build.log')
    run_command(['ctest','--test-dir','core/build','-V','--output-on-failure'],output/'cpp_tests.log')
    cpp_tests = [dict(name=name,status=status) for status,name in
                 re.findall(r'\[(PASS|FAIL)\]\s+([A-Za-z_]\w*)',(output/'cpp_tests.log').read_text())]
    (output/'cpp_test_status.json').write_text(json.dumps(cpp_tests,indent=2))
    junit = output/'pytest.xml'
    run_command([sys.executable,'-m','pytest','-p','no:anyio','-q','--junitxml',str(junit)],output/'python_tests.log')
    test_status = []
    for case in ET.parse(junit).iter('testcase'):
        test_status.append(dict(name=case.attrib['name'],classname=case.attrib.get('classname',''),
                                status='FAIL' if case.find('failure') is not None or case.find('error') is not None else 'PASS'))
    (output/'test_status.json').write_text(json.dumps(test_status,indent=2))
    from scripts.validate_physical_events import run_validation
    physical = run_validation(output/'invariants')
    if not physical['passed']:
        raise RuntimeError('Physical invariants failed')
    import numpy as np
    import yaml
    from mujoco_sim.python.sim_runner import SimulationRunner
    from mujoco_sim.python.recorder import save_csv
    config = yaml.safe_load((ROOT/'configs'/'physics_validation.yaml').read_text())['validation']
    summary = dict(stage='P0_P1',learning_enabled=False,cases={},passed=False,completed=False,
                   expected_case_count=len(config['cases']),completed_case_count=0)
    for name,options in config['cases'].items():
        path = output/name; path.mkdir(parents=True,exist_ok=True)
        runner = SimulationRunner(zero_preload=name != 'preloaded_capture_release')
        resolved = dict(runner.config,validation=config,scenario=options,release_spring_enabled=not runner.latch.zero_preload)
        (path/'config.json').write_text(json.dumps(resolved,indent=2))
        try:
            rows = runner.run_scenario(**options)
            save_csv(path/'timeseries.csv',rows)
            (path/'events.json').write_text(json.dumps(runner.events,indent=2))
            tail = [r for r in rows if r['time'] >= rows[-1]['time']-config['tail_window_s']]
            peak_position = max(r[f'q{i}_pos_error_m'] for r in tail for i in (1,2))
            peak_speed = max(np.linalg.norm([r[f'q{i}_velocity_{a}'] for a in 'xyz']) for r in tail for i in (1,2))
            peak_attitude = max(np.degrees(r[f'q{i}_attitude_error_rad']) for r in tail for i in (1,2))
            actual = np.array([[r[f'motor{k}_actual_N'] for k in range(8)] for r in rows])
            desired = np.array([[r[f'motor{k}_command_N'] for k in range(8)] for r in rows])
            ticks = [r for r in rows if r['control_updated']]
            metrics = dict(tail_position_error_m=float(peak_position),tail_speed_mps=float(peak_speed),
                           tail_attitude_error_deg=float(peak_attitude),physics_steps=len(rows),control_ticks=len(ticks),
                           actual_control_rate_hz=len(ticks)/rows[-1]['time'],dock_time_s=runner.dock_event_time,
                           first_contact_time_s=runner.first_contact_time,
                           release_time_s=runner.release_event_time,
                           peak_contact_normal_force_N=max(r['contact_normal_force_N'] for r in rows),
                           integrated_normal_contact_impulse_est_Ns=rows[-1]['contact_impulse_est_Ns'],
                           peak_actual_motor_slew_Nps=float(np.max(np.abs(np.diff(actual,axis=0)))/runner.dt_sim),
                           motor_command_saturation_fraction=float(np.mean((desired<=.05+1e-9)|(desired>=5.5-1e-9))),
                           event_instantaneous_max_velocity_change=max([max(abs(np.array(e['post']['qvel'])-e['pre']['qvel'])) for e in runner.events] or [0.]))
            gates = dict(finite_state=all(np.isfinite(r[f'q{i}_{a}']) for r in rows for i in (1,2) for a in 'xyz'),
                         stable_tail=peak_position<config['max_tail_position_error_m'] and peak_speed<config['max_tail_speed_mps'] and peak_attitude<config['max_tail_attitude_error_deg'],
                         motor_limits=bool(actual.min()>=0. and actual.max()<=5.5),
                         motor_slew=metrics['peak_actual_motor_slew_Nps']<=45.+1e-8,
                         exact_control_count=len(ticks)==round(options['duration']/runner.control_dt),
                         no_state_teleport=metrics['event_instantaneous_max_velocity_change']==0.)
            if runner.release_event_time is not None:
                pre = [r for r in rows if r['time'] <= runner.release_event_time+1e-10][-1]
                release_rows = [r for r in rows if 0 <= r['time']-runner.release_event_time <= 1.2]
                release_error = max(np.degrees(r[f'q{i}_attitude_error_rad']) for r in release_rows for i in (1,2))
                internal_torque = max(np.linalg.norm([pre[f'q{i}_rotor_torque_body_{a}'] for a in 'xyz']) for i in (1,2))
                metrics.update(release_peak_true_attitude_error_deg=float(release_error),
                               prerelease_max_rotor_torque_Nm=float(internal_torque),
                               prerelease_contact_normal_force_N=pre['contact_normal_force_N'],
                               prerelease_weld_force_N=float(np.linalg.norm([pre[f'weld_q1_force_world_N_{a}'] for a in 'xyz'])))
                gates['release_transient'] = release_error < config['max_release_attitude_error_deg']
                gates['no_large_internal_control_preload'] = internal_torque < config['max_prerelease_rotor_torque_Nm']
            if name == 'missed_capture':
                gates['no_false_capture'] = runner.dock_event_time is None and not any(r['weld_active'] for r in rows)
            elif not options.get('preconnected') and name != 'independent_hover':
                gates['capture_and_release'] = runner.dock_event_time is not None and runner.release_event_time is not None
            if options.get('preconnected'):
                gates['connected_throughout'] = all(r['weld_active'] for r in rows)
            gates = {key:bool(value) for key,value in gates.items()}
            passed = all(gates.values())
            record = dict(status='PASS' if passed else 'FAIL',gates=gates,metrics=metrics)
            plot_case(path,rows,runner.events)
        except Exception as exc:
            record = dict(status='FAIL',error=repr(exc))
            if runner.telemetry:
                save_csv(path/'timeseries.csv',runner.telemetry)
        summary['cases'][name] = record
        summary['completed_case_count'] = len(summary['cases'])
        summary['completed'] = summary['completed_case_count']==summary['expected_case_count']
        summary['passed'] = summary['completed'] and all(r['status']=='PASS' for r in summary['cases'].values())
        (path/'summary.json').write_text(json.dumps(record,indent=2))
        print(f"{record['status']}: {name}",flush=True)
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
    if not summary['passed']:
        raise SystemExit(1)
    print(f'P0/P1 acceptance passed; artifacts: {output}',flush=True)


if __name__ == '__main__':
    main()
