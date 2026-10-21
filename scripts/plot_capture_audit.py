#!/usr/bin/env python3
"""Plots exclusively recorded MuJoCo samples; no synthetic response curves."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    with path.open() as f: return list(csv.DictReader(f))


def col(rows,key):
    return np.array([float(r[key]) if r.get(key) not in ('',None) else np.nan for r in rows])


def events(path): return json.loads(path.read_text())


def plot(study,baseline,output):
    output.mkdir(parents=True,exist_ok=True)
    rows=load(study/'nominal/timeseries.csv');old=load(baseline/'nominal/timeseries.csv')
    ev=events(study/'nominal/events.json');old_ev=events(baseline/'nominal/events.json')
    contact=next(e['time'] for e in ev if e['event']=='CONTACT')
    dock=next(e['time'] for e in ev if e['event']=='DOCK')
    t=col(rows,'time')
    fig,axes=plt.subplots(4,2,figsize=(14,13))
    for data,label in ((old,'Baseline 176b02c'),(rows,'Finite-anchor latch')):
        axes[0,0].plot(col(data,'time'),col(data,'rel_dist')*1000,label=label)
    axes[0,0].set_ylabel('Connector distance (mm)');axes[0,0].set_ylim(0,12)
    axes[0,1].plot(t,col(rows,'relative_normal_velocity_mps'),label='Face normal velocity (m/s)')
    axes[0,1].plot(t,col(rows,'relative_angular_speed_radps'),label='Relative angular speed (rad/s)')
    axes[0,1].set_ylabel('Relative velocities')
    for key,label in [('contact_normal_force_N','Current contact normal'),('capture_static_force_N','Static tension'),
                      ('capture_damping_force_N','Effective dashpot'),('capture_resultant_force_N','Resultant tension')]:
        axes[1,0].plot(t,col(rows,key),label=label)
    axes[1,0].set_ylabel('Force (N)')
    axes[1,1].step(t,col(rows,'mode'),where='post',label='FSM mode')
    axes[1,1].step(t,col(rows,'capture_active'),where='post',label='Capture engaged')
    axes[1,1].step(t,col(rows,'weld_active')*4,where='post',label='Weld x4')
    axes[1,1].set_ylabel('State timeline')
    axes[2,0].plot(t,col(rows,'weld_anchor_error_m')*1e6,label='Weld anchor error (um)')
    axes[2,0].set_ylabel('Position constraint error (um)')
    ax=axes[2,0].twinx();ax.plot(t,np.degrees(col(rows,'weld_attitude_error_rad')),color='orange',label='Attitude constraint error')
    ax.set_ylabel('Attitude constraint error (deg)')
    axes[2,1].plot(t,np.linalg.norm(np.column_stack([col(rows,'weld_q1_force_world_N_'+a) for a in 'xyz']),axis=1),label='Equality-only weld force (N)')
    axes[2,1].plot(t,np.linalg.norm(np.column_stack([col(rows,'weld_q1_couple_at_anchor_world_Nm_'+a) for a in 'xyz']),axis=1),label='Anchor couple (Nm)')
    axes[2,1].set_ylabel('Internal constraint loads')
    for k in range(8): axes[3,0].plot(t,col(rows,f'motor{k}_actual_N'),lw=.6)
    axes[3,0].set_ylabel('Eight actual motor thrusts (N)')
    axes[3,1].plot(t,col(rows,'capture_damping_power_W'),label='Damper power (W)')
    axes[3,1].plot(t,col(rows,'capture_dissipated_energy_J')*1000,label='Dissipation (mJ)')
    axes[3,1].set_ylabel('Capture energy accounting')
    for ax in axes.flat:
        ax.grid(alpha=.2);ax.set_xlabel('Simulation time (s)')
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=7)
        for e in ev:
            if e['event'] in ('CONTACT','CAPTURE_START','CAPTURE_READY','DOCK','RELEASE_PREP','UNDOCK'):
                ax.axvline(e['time'],color='k',alpha=.2,ls=':')
    fig.tight_layout();fig.savefig(output/'nominal_full.png',dpi=140)
    for ax in axes.flat: ax.set_xlim(contact-.08,dock+.15)
    fig.savefig(output/'capture_window.png',dpi=150);plt.close(fig)
    summary=json.loads((study/'summary.json').read_text())
    fig,axs=plt.subplots(1,2,figsize=(12,4))
    seeds=sorted({r['seed'] for r in summary['trials'] if r['seed'] is not None})
    cats=['safe','boundary','reject'];colors=['tab:green','tab:orange','tab:red']
    for j,cat in enumerate(cats):
        rates=[]
        for seed in seeds:
            data=[r for r in summary['trials'] if r['category']==cat and r['seed']==seed]
            rates.append(sum(r['metrics']['capture_success'] for r in data)/len(data))
        axs[0].bar(np.arange(len(seeds))+.25*(j-1),rates,.24,label=cat,color=colors[j])
        data=[r['metrics'] for r in summary['trials'] if r['category']==cat and r['seed'] is not None]
        observed=[r for r in data if r['max_postcontact_distance_m'] is not None]
        axs[1].scatter([r['peak_contact_normal_N'] for r in observed],
                       [r['max_postcontact_distance_m']*1000 for r in observed],label=f'{cat}: {len(data)} retained',color=colors[j],alpha=.7)
    axs[0].set_xticks(np.arange(len(seeds)),[str(s) for s in seeds]);axs[0].set_ylabel('Capture fraction');axs[0].set_ylim(0,1.1)
    axs[1].set_xlabel('Peak contact force (N)');axs[1].set_ylabel('Max contact-to-lock/end gap (mm)')
    for ax in axs:ax.legend();ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'seeded_outcomes.png',dpi=150);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4))
    for name in ('damping_only','tensile_capture','tensile_refined'):
        data=load(study/(name+'.csv'));time=col(data,'time')
        budget=col(data,'kinetic_J')+col(data,'capture_potential_J')+col(data,'dissipated_J')+col(data,'discarded_J')+col(data,'reservoir_J')
        axs[0].plot(time,(budget-summary['independent'][name]['initial_total_energy_J'])*1e6,label=name)
        axs[1].plot(time,col(data,'angular_error'),label=name)
    axs[0].set_ylabel('Energy balance residual (uJ)');axs[1].set_ylabel('Angular momentum error (kg m2/s)')
    for ax in axs:ax.set_xlabel('Unpowered experiment time (s)');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(output/'independent_physics.png',dpi=150);plt.close(fig)
    if (study/'high_speed_failure').exists():
        data=load(study/'high_speed_failure/timeseries.csv');ev=events(study/'high_speed_failure/events.json');time=col(data,'time')
        fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
        axs[0].plot(time,col(data,'rel_dist')*1000,label='Distance (mm)');axs[0].set_ylabel('mm')
        axs[1].plot(time,col(data,'relative_normal_velocity_mps'),label='Normal velocity (m/s)')
        axs[1].plot(time,col(data,'relative_angular_speed_radps'),label='Angular speed (rad/s)')
        axs[2].plot(time,col(data,'contact_normal_force_N'),label='Contact (N)')
        axs[2].plot(time,col(data,'capture_resultant_force_N'),label='Capture tension (N)')
        for ax in axs:
            for e in ev:
                if e['event'] in ('CONTACT','CAPTURE_FAILED'):ax.axvline(e['time'],color='k',alpha=.4,ls=':')
            ax.grid(alpha=.2);ax.legend()
        axs[2].set_xlabel('Physical time (s); no DOCK event')
        fig.tight_layout();fig.savefig(output/'high_speed_failure.png',dpi=150);plt.close(fig)
    records=summary['trials']
    fields=['id','seed','category','expectation']+list(records[0]['metrics'])+['exception','early_termination']
    with (output/'trial_statistics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in records:writer.writerow({k:v for k,v in r.items() if k in fields}|r['metrics'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--study',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();plot(a.study,a.baseline,a.output)
