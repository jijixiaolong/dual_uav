"""Small artifact helpers shared by real MuJoCo research experiments."""
import csv
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np


def jsonable(value):
    if isinstance(value,dict): return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [jsonable(v) for v in value]
    if isinstance(value,np.ndarray): return jsonable(value.tolist())
    if isinstance(value,np.generic): return jsonable(value.item())
    if isinstance(value,float) and not np.isfinite(value): return None
    return value


def write_json(path,value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(jsonable(value),indent=2,allow_nan=False)+'\n')


def save_rows(path,rows):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if not rows:
        path.write_text('');return
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'wt',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def load_rows(path):
    path=Path(path); opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt',newline='') as f:
        out=[]
        for r in csv.DictReader(f):
            for key,value in r.items():
                if value=='': r[key]=None
                else:
                    try:r[key]=float(value)
                    except ValueError:pass
            out.append(r)
    return out


def digest(value):
    return hashlib.sha256(json.dumps(jsonable(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
