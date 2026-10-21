#!/usr/bin/env python3
"""Paired release interventions; no nominal plant/controller implementation is replaced."""
import argparse
import hashlib
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from mujoco_sim.python.sim_runner import SimulationRunner
from analysis_tools.metrics import evaluate_window
from analysis_tools.research_io import save_rows,write_json,digest

VARIANTS={
 'standard':{},
 'weld_kept_frozen':dict(release_constraint=False,switch_controller=False,freeze_motor_commands=True),
 'released_frozen':dict(switch_controller=False,freeze_motor_commands=True),
 'weld_kept_aggregate':dict(release_constraint=False,switch_controller=False),
 'released_aggregate':dict(switch_controller=False),
 'weld_kept_independent_hold':dict(release_constraint=False,release_reference='hold'),
 'released_independent_hold':dict(release_reference='hold'),
 'standard_no_lag_after_release':dict(release_motor_tau_s=0.),
 'hold_no_lag_after_release':dict(release_reference='hold',release_motor_tau_s=0.),
 'standard_preserve_integrators':dict(reset_integrators=False),
 'standard_hover_allocator_reset':dict(allocator_state='hover_reset'),
 'standard_slow_separation':dict(separation_duration_s=3.),
}


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    protected=['mujoco_sim/python/mujoco_world.py','mujoco_sim/assets/dual_quads.xml','core/src/geometric_se3.cpp','core/src/allocator.cpp']
    manifest=dict(scenario=dict(duration=12.,dock_cmd_time=1.,undock_cmd_time=6.),
                  interventions=VARIANTS,paired_until='immediately before the SEPARATING transition',
                  protected_source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in protected})
    write_json(output/'config.json',manifest)
    records=[]; traces={};checkpoints={}
    for preload in (False,True):
        for name,options in VARIANTS.items():
            # Preload is a separate paired block; its pre-event state is not
            # asserted equal to the uncharged block.
            case=f"{'spring' if preload else 'no_spring'}__{name}"
            runner=SimulationRunner(zero_preload=not preload)
            rows=runner.run_scenario(**manifest['scenario'],research_options=options)
            cp=runner.research_release_checkpoint
            if cp is None:raise RuntimeError(f'No intervention anchor in {case}')
            anchor=cp['time']; checkpoint_hash=digest(cp)
            checkpoint_key='spring' if preload else 'no_spring'
            if checkpoint_key not in checkpoints:checkpoints[checkpoint_key]=checkpoint_hash
            if checkpoint_hash!=checkpoints[checkpoint_key]:
                raise AssertionError(f'Pre-intervention state differs in {case}')
            analysis=evaluate_window(rows,anchor,1.2,config={'expected_events':[]})
            long_analysis=evaluate_window(rows,anchor,3.,config={'expected_events':[]})
            pre=next(r for r in reversed(rows) if r['time']<=anchor+1e-10)
            first=next(r for r in rows if r['time']>anchor+1e-10)
            reference_jumps=[]
            for i in (1,2):
                qa=np.array([pre[f'q{i}_desired_quat_{c}'] for c in 'wxyz'])
                qb=np.array([first[f'q{i}_desired_quat_{c}'] for c in 'wxyz'])
                reference_jumps.append(float(np.degrees(2*np.arccos(np.clip(abs(qa@qb),0,1)))))
            record=dict(case=case,preload=preload,variant=name,options=options,anchor_time_s=anchor,
                        physical_release=options.get('release_constraint',True),
                        pre_intervention_sha256=checkpoint_hash,prefix_identical=True,
                        active_reference_first_tick_change_deg=reference_jumps,
                        window=analysis['window'],metrics=analysis['metrics'],three_second_metrics=long_analysis['metrics'],
                        prerelease={k:pre[k] for k in pre if k.startswith('weld_') or k.startswith('spring_') or 'rotor_torque_body' in k})
            path=output/case;path.mkdir(exist_ok=True)
            save_rows(path/'timeseries.csv.gz',rows)
            write_json(path/'events.json',runner.events);write_json(path/'checkpoint.json',cp)
            write_json(path/'metrics.json',record)
            records.append(record)
            traces[case]=[r for r in rows if anchor-.10<=r['time']<=anchor+1.2]
            print(case,'peak_deg',analysis['metrics']['combined']['peak_attitude_error_deg'],flush=True)
    by={r['case']:r for r in records}
    contrasts=[]
    for prefix in ('no_spring','spring'):
        for label,a,b in [('mechanical_frozen','weld_kept_frozen','released_frozen'),
                          ('mechanical_aggregate_feedback','weld_kept_aggregate','released_aggregate'),
                          ('mechanical_independent_hold','weld_kept_independent_hold','released_independent_hold'),
                          ('controller_switch_released_hold','released_aggregate','released_independent_hold'),
                          ('controller_switch_weld_kept','weld_kept_aggregate','weld_kept_independent_hold'),
                          ('reference_separation','released_independent_hold','standard'),
                          ('motor_lag','standard_no_lag_after_release','standard'),
                          ('integrator_reset','standard_preserve_integrators','standard'),
                          ('allocation_history','standard_hover_allocator_reset','standard'),
                          ('trajectory_duration','standard_slow_separation','standard')]:
            A=by[f'{prefix}__{a}']['metrics']['combined'];B=by[f'{prefix}__{b}']['metrics']['combined']
            contrasts.append(dict(block=prefix,contrast=label,from_variant=a,to_variant=b,
                                  paired_differences={k:B[k]-A[k] for k in ('peak_attitude_error_deg','peak_rate_deg_s','max_position_error_m','attitude_error_integral_deg_s','position_error_integral_m_s')}))
    write_json(output/'summary.json',dict(passed=True,paired_blocks=2,case_count=len(records),records=records,contrasts=contrasts,
                 caveat='Deterministic causal interventions in one nominal scenario; nonlinear contrasts are not additive causal percentages. Retained-weld variants are diagnostic counterfactuals, not production baselines.'))
    save_rows(output/'paired_contrasts.csv',[{'block':x['block'],'contrast':x['contrast'],**x['paired_differences']} for x in contrasts])
    plot(output,traces,by)
    return records


def plot(output,traces,by):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for prefix in ('no_spring','spring'):
        fig,axes=plt.subplots(4,2,figsize=(13,12),sharex=True)
        selected=('standard','released_independent_hold','released_aggregate','released_frozen','weld_kept_frozen','standard_slow_separation')
        for name in selected:
            rows=traces[f'{prefix}__{name}']; t=np.array([r['time']-by[f'{prefix}__{name}']['anchor_time_s'] for r in rows])
            for col,i in enumerate((1,2)):
                axes[0,col].plot(t,[r[f'q{i}_roll_deg'] for r in rows],label=name,linewidth=1)
                axes[1,col].plot(t,[r[f'q{i}_roll_error_deg'] for r in rows],linewidth=1)
                axes[2,col].plot(t,[np.linalg.norm([r[f'q{i}_omega_body_{a}'] for a in 'xyz']) for r in rows],linewidth=1)
                axes[3,col].plot(t,[r[f'q{i}_rotor_torque_body_x'] for r in rows],linewidth=1)
        for j,label in enumerate(('True roll (deg)','Active roll error (deg)','Angular speed (rad/s)','Actual motor roll torque (N m)')):
            for i in (0,1):axes[j,i].set_ylabel(f'Q{i+1} {label}')
        axes[0,0].legend(fontsize=7)
        for ax in axes.flat:ax.grid(alpha=.25);ax.axvline(0,color='k',linestyle=':')
        for ax in axes[-1]:ax.set_xlabel('Time since intervention (s)')
        fig.suptitle(f'Paired release interventions: {prefix}');fig.tight_layout();fig.savefig(output/f'{prefix}_paired_response.png',dpi=160);plt.close(fig)
        rows=traces[f'{prefix}__standard'];t=[r['time']-by[f'{prefix}__standard']['anchor_time_s'] for r in rows]
        fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
        for col,i in enumerate((1,2)):
            axes[0,col].plot(t,[r[f'q{i}_roll_deg'] for r in rows],label='actual roll')
            # In the present low-pitch scenario desired roll is recovered directly
            # from the recorded desired quaternion, not from error subtraction.
            desired=[]
            for r in rows:
                w,x,y,z=[r[f'q{i}_desired_quat_{c}'] for c in 'wxyz']
                desired.append(np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))))
            axes[0,col].plot(t,desired,label='active desired roll')
            axes[1,col].plot(t,[r[f'q{i}_desired_torque_Nm_x'] for r in rows],label='desired roll torque')
            axes[1,col].plot(t,[r[f'q{i}_rotor_torque_body_x'] for r in rows],label='actual motor roll torque')
            for k in range(4):axes[2,col].plot(t,[r[f'motor{4*(i-1)+k}_command_N'] for r in rows],linewidth=.8)
        for ax in axes.flat:ax.grid(alpha=.25);ax.axvline(0,color='k',linestyle=':')
        for ax in axes[0]:ax.set_ylabel('Angle (deg)');ax.legend(fontsize=8)
        for ax in axes[1]:ax.set_ylabel('Torque (N m)');ax.legend(fontsize=8)
        for ax in axes[2]:ax.set_ylabel('Four motor commands (N)');ax.set_xlabel('Time since release (s)')
        fig.tight_layout();fig.savefig(output/f'{prefix}_reference_and_control.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'results/event_attribution/release')
    run(parser.parse_args().output)
