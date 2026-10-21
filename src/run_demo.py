"""Run a reproducible SURROGATE study; results are synthetic by construction."""
import argparse
import json
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
try:
    from .event_surrogate import evaluate, fit_lifted, sample_feature, run, choose_c
except ImportError:
    from event_surrogate import evaluate, fit_lifted, sample_feature, run, choose_c


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=Path('results'))
    p.add_argument('--n-train', type=int, default=40)
    p.add_argument('--n-test', type=int, default=100)
    p.add_argument('--seed', type=int, default=4)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    methods = ('baseline', 'fixed_ff', 'event_learning')
    rows = []
    for mode in ('dock', 'undock'):
        outcomes, (A,G) = evaluate(mode, args.n_train, args.seed, args.n_test)
        for trial, outcome in enumerate(outcomes):
            for method in methods:
                rows.append(dict(mode=mode, trial=trial, method=method, **outcome[method]))
        rng = np.random.default_rng(args.seed+900)
        feat = sample_feature(rng, mode, test=True)
        nominal = np.array([.10, 0., 0.]) if mode=='dock' else np.zeros(3)
        variants={'baseline':np.zeros(2),'fixed_ff':choose_c(mode, nominal, A, G), 'event_learning':choose_c(mode,feat,A,G)}
        fig, ax = plt.subplots(figsize=(8.5,4.3))
        for name, coeff in variants.items():
            t,theta,omega,u=run(mode,feat,coeff)
            ax.plot(t,np.rad2deg(theta),label=name)
        ax.set(title=f'{mode}: event response (synthetic surrogate)', xlabel='Time since event (s)', ylabel='Pitch error (degrees)')
        ax.grid(alpha=.2); ax.legend();fig.tight_layout();fig.savefig(args.output/f'{mode}_example.png',dpi=160);plt.close(fig)
    with (args.output/'synthetic_trials.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader();writer.writerows(rows)
    summary={}
    for mode in ('dock','undock'):
        summary[mode]={}
        for method in methods:
            r=[x for x in rows if x['mode']==mode and x['method']==method]
            summary[mode][method]={metric:round(float(np.mean([x[metric] for x in r])),7)
                for metric in ('score','peak_theta_deg','peak_rate_deg_s','input_energy')}
    (args.output/'summary.json').write_text(json.dumps({'disclaimer':'synthetic surrogate, not physical contact simulation', 'summary':summary},indent=2),encoding='utf-8')
    for mode in ('dock','undock'):
        print(mode)
        for method in methods:
            print(method,summary[mode][method])
    print('All results are synthetic, illustrative and not usable as experimental evidence.')

if __name__ == '__main__': main()
