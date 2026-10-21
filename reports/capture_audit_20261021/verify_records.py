#!/usr/bin/env python3
"""Independently reconstruct CSV modes and audit every recorded DOCK event."""
import csv
import gzip
import hashlib
import itertools
import json
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from analysis_tools.capture_trials import row_eligible,snapshot_eligible


def verify():
    directory=Path(__file__).resolve().parent
    data=directory/'data'
    events=json.loads((data/'events.json').read_text())
    criteria=yaml.safe_load((ROOT/'configs/docking_mechanism.yaml').read_text())['docking']['latch_criteria']
    text_fields={'case_id','category','seed','mode_name','phase','contact_state'}
    results=[]
    with gzip.open(data/'capture_traces.csv.gz','rt',newline='') as f:
        for name,source in itertools.groupby(csv.DictReader(f),key=lambda r:r['case_id']):
            rows=[{k:(v if k in text_fields else float(v) if v!='' else None) for k,v in r.items()} for r in source]
            es=events[name]
            transitions=[e for e in es if e['event']=='FSM_TRANSITION']
            mismatch=0
            for r in rows:
                # Post-step CSV: a transition at t first appears in row t+dt.
                prior=[e for e in transitions if e['time']<r['time']-1e-10]
                mode=prior[-1]['to_mode'] if prior else 'FREE'
                mismatch+=int(mode!=r['mode_name'])
            jump=0.
            bad_locks=0
            for e in es:
                for key in ('qpos','qvel'):
                    jump=max(jump,float(np.max(np.abs(np.asarray(e['post'][key])-e['pre'][key]))))
                if e['event']=='DOCK':
                    pre=[r for r in rows if e['time']-criteria['dwell_time_s']-1e-10<=r['time']<=e['time']+1e-10]
                    bad_locks+=int(not snapshot_eligible(e['pre'],criteria) or any(not row_eligible(r,criteria) for r in pre))
            dt=np.diff([r['time'] for r in rows])
            cadence=bool(np.allclose(dt,.002,atol=1e-12,rtol=0))
            results.append(dict(case_id=name,rows=len(rows),events=len(es),mode_mismatch_samples=mismatch,
                invalid_lock_events=bad_locks,max_instantaneous_qpos_qvel_jump=jump,physics_cadence_valid=cadence,
                passed=mismatch==0 and bad_locks==0 and jump==0. and cadence))
    output=dict(passed=all(r['passed'] for r in results),cases=len(results),rows=sum(r['rows'] for r in results),
        events=sum(r['events'] for r in results),results=results,
        inputs_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (data/'capture_traces.csv.gz',data/'events.json')})
    (directory/'record_verification.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({k:v for k,v in output.items() if k!='results'},indent=2))
    if not output['passed']:raise SystemExit(1)


if __name__=='__main__':verify()
