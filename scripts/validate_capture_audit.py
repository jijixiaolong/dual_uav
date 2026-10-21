#!/usr/bin/env python3
"""Run a retained, seeded physical capture study. Does not train any controller."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
from importlib.metadata import version
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from analysis_tools.capture_trials import generate_trials,trial_metrics,DEFAULT_SEEDS
from analysis_tools.capture_experiments import capture_experiment
from mujoco_sim.python.sim_runner import SimulationRunner
from mujoco_sim.python.recorder import save_csv
from mujoco_sim.python.event_interfaces import event_feature_extractor,event_response_recorder,event_metrics
from scripts.validate_stage2 import source_hashes


def write(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def aggregate(records):
    result={}
    for category in ('safe','boundary','reject','targeted'):
        selected=[r for r in records if r['category']==category]
        if not selected: continue
        values=[r['metrics'] for r in selected]
        stats={}
        for key in ('contact_to_lock_s','max_postcontact_distance_m','peak_contact_normal_N',
                    'peak_internal_weld_force_N','max_weld_anchor_error_m','max_weld_attitude_error_rad',
                    'motor_saturation_fraction','peak_motor_command_slew_Nps'):
            observed=[r[key] for r in values if r[key] is not None]
            stats[key]=dict(observed_count=len(observed),mean=float(np.mean(observed)) if observed else None,
                min=min(observed,default=None),max=max(observed,default=None))
        n=len(selected)
        result[category]=dict(trials=n,captured=sum(r['capture_success'] for r in values),
            success_rate=sum(r['capture_success'] for r in values)/n,
            false_locks=sum(r['false_lock_count'] for r in values),
            false_lock_trial_rate=sum(r['false_lock_count']>0 for r in values)/n,
            unexpected_reject_locks=sum(r['capture_success'] for r in values) if category=='reject' else 0,
            exceptions=sum(r['exception'] is not None for r in selected),
            premature_terminations=sum(r['early_termination'] for r in selected),statistics=stats)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=list(DEFAULT_SEEDS))
    p.add_argument('--samples-per-class',type=int,default=6);args=p.parse_args()
    if args.samples_per_class<1: raise ValueError('Positive sample count required')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    manifest=dict(git_base_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        git_diff_sha256=__import__('hashlib').sha256(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT)).hexdigest(),
        source_sha256=source_hashes(),python=sys.version,platform=platform.platform(),
        packages={x:version(x) for x in ('mujoco','numpy','scipy','pybind11','pytest','matplotlib')},
        random_seeds=args.seeds,rng='numpy.random.default_rng / PCG64',samples_per_class=args.samples_per_class,
        outcomes_not_conditioned_on_success=True,
        contact_parameter_note='Solver-effective stiffness/damping RATIOS via positive solref; no calibrated material constants.',
        model_note='Finite post-contact tensile latch; nominal C++ SE(3) and allocation unchanged; plant mass/inertia mismatched intentionally.')
    write(output/'manifest.json',manifest)
    scenarios=[]
    for seed in args.seeds: scenarios.extend(generate_trials(seed,args.samples_per_class))
    targeted=[
        ('nominal',dict(duration=10.,dock_cmd_time=1.,undock_cmd_time=5.)),
        ('target_gap_30mm',dict(duration=6.,dock_cmd_time=1.,undock_cmd_time=None,approach_offset=.03)),
        ('miss_60mm',dict(duration=6.,dock_cmd_time=1.,undock_cmd_time=None,approach_offset=.06)),
        ('high_speed_collision',dict(duration=1.,dock_cmd_time=0.,undock_cmd_time=None,q1_init_y=-.225,q2_init_y=.225,
            research_options=dict(q1_initial_state=dict(velocity_world_mps=[0,.8,0]),q2_initial_state=dict(velocity_world_mps=[0,-.8,0])))),
        ('high_speed_failure',dict(duration=1.,dock_cmd_time=0.,undock_cmd_time=None,q1_init_y=-.225,q2_init_y=.225,
            research_options=dict(q1_initial_state=dict(velocity_world_mps=[0,2.,0]),q2_initial_state=dict(velocity_world_mps=[0,-2.,0])))),
        ('high_angular_collision',dict(duration=1.,dock_cmd_time=0.,undock_cmd_time=None,q1_init_y=-.2199,q2_init_y=.2199,
            research_options=dict(q2_initial_state=dict(omega_body_radps=[0,0,4])))),
        ('edge_contact',dict(duration=1.,dock_cmd_time=0.,undock_cmd_time=None,q1_init_y=-.2199,q2_init_y=.2199,
            research_options=dict(q2_initial_offset=[.038,0,0]))),
    ]
    scenarios += [dict(id=name,seed=None,category='targeted',expectation='inspect_physical_history',scenario=options)
                  for name,options in targeted]
    write(output/'trial_design.json',scenarios)
    records=[]
    runner=SimulationRunner()  # Reuse intentionally; reset independently tested.
    for trial in scenarios:
        path=output/trial['id'];path.mkdir()
        write(path/'config.json',trial)
        exception=None
        try:
            rows=runner.run_scenario(**trial['scenario'])
        except Exception:
            exception=traceback.format_exc();rows=runner.telemetry
        events=runner.latch.events
        if rows: save_csv(path/'timeseries.csv',rows)
        write(path/'events.json',events)
        metrics=trial_metrics(rows,events,runner.latch.params)
        record=dict(id=trial['id'],seed=trial['seed'],category=trial['category'],expectation=trial['expectation'],
            metrics=metrics,exception=exception,
            early_termination=len(rows)!=round(trial['scenario']['duration']/runner.dt_sim),
            realized_plant=dict(mass_kg=runner.world.model.body_mass[runner.world.body_ids].tolist(),
                inertia_kg_m2=runner.world.model.body_inertia[runner.world.body_ids].tolist(),
                pad_solref=runner.world.model.geom_solref[list(sorted(runner.world.connector_geoms))].tolist(),
                capture_stiffness_Npm=runner.latch.capture_stiffness,capture_damping_Nspm=runner.latch.capture_damping,
                motor_tau_s=runner.world.motor_tau))
        write(path/'summary.json',record);records.append(record)
        if trial['id']=='nominal':
            write(path/'pre_event_features.json',[asdict(event_feature_extractor(e)) for e in events])
            responses=event_response_recorder(rows,events)
            write(path/'response_windows.json',[{k:v for k,v in r.items() if k!='rows'}|{'samples':len(r['rows'])} for r in responses])
            write(path/'event_metrics.json',[dict(event=e['event'],time=e['time'],result=event_metrics(rows,e))
                for e in events if e['event'] in ('CONTACT','DOCK','UNDOCK')])
        print(f"{trial['id']}: captured={metrics['capture_success']} false={metrics['false_lock_count']} exception={exception is not None}",flush=True)
        write(output/'summary.json',dict(groups=aggregate(records),trials=records))
    independent={}
    for name,dt,damping_only in [('damping_only',.0005,True),('tensile_capture',.0005,False),('tensile_refined',.00025,False)]:
        w,l,initial,rows=capture_experiment(dt,damping_only)
        save_csv(output/(name+'.csv'),rows)
        accounted=[r['kinetic_J']+r['capture_potential_J']+r['dissipated_J']+r['discarded_J']+r['reservoir_J'] for r in rows]
        independent[name]=dict(dt_s=dt,initial_kinetic_J=initial[2],final_kinetic_J=rows[-1]['kinetic_J'],
            max_linear_momentum_error=max(r['linear_error'] for r in rows),
            max_angular_momentum_error=max(r['angular_error'] for r in rows),
            initial_total_energy_J=initial[2]+initial[3]+initial[4],
            max_energy_accounting_excess_J=max(accounted)-sum(initial[2:]),
            dissipated_J=rows[-1]['dissipated_J'],max_damping_power_W=max(r['power_W'] for r in rows))
    write(output/'independent_physics.json',independent)
    grouped=aggregate(records)
    write(output/'summary.json',dict(groups=grouped,trials=records,independent=independent))
    # Fail on actual implementation/physical guards, not every unsuccessful capture.
    physical_ok=all(v['max_linear_momentum_error']<1e-10 and v['max_angular_momentum_error']<2e-6
        and v['max_energy_accounting_excess_J']<2e-5 and v['max_damping_power_W']<=1e-12 for v in independent.values())
    if (any(g['false_locks'] or g['exceptions'] or g['premature_terminations'] for g in grouped.values())
            or grouped['reject']['unexpected_reject_locks'] or not physical_ok):
        raise SystemExit(1)


if __name__=='__main__': main()
