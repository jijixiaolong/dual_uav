#!/usr/bin/env python3
"""Unified experiment orchestration and evaluation matrix for Dual-Quadrotor MuJoCo simulation."""
import argparse
import json
import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / 'core' / 'build'))

from mujoco_sim.python.sim_runner import SimulationRunner
import docking_core_py as dcp
from offline_learning.fit_eclr import (
    sample_pre_event_conditions,
    collect_eclr_dataset,
    fit_eclr_matrices,
    compute_fixed_feedforward
)
from offline_learning.ilc_baseline import EventAlignedILC
from offline_learning.evaluate_splits import evaluate_trial


def run_smoke_test():
    print("=" * 65)
    print("Running Smoke Test: 6-DoF Dual Quadrotor Docking in MuJoCo...")
    print("=" * 65)
    runner = SimulationRunner()
    logs = runner.run_scenario(duration=12.0, dock_cmd_time=1.0, undock_cmd_time=6.0)

    modes = sorted(list(set(l['mode_name'] for l in logs)))
    weld_count = sum(1 for l in logs if l['weld_active'])
    print(f"[OK] Simulation completed {len(logs)} steps over {logs[-1]['time']:.2f} seconds.")
    print(f"[OK] Modes traversed: {modes}")
    print(f"[OK] Weld constraint active steps: {weld_count}")
    assert 'CONNECTED' in modes, "Failed to enter CONNECTED mode in smoke test"
    assert 'SEPARATING' in modes, "Failed to enter SEPARATING mode in smoke test"
    print("\n>>> Smoke test passed successfully! <<<\n")


def run_full_matrix(output_dir: Path, n_train: int = 12, n_test: int = 15, seed: int = 42):
    raise RuntimeError('Legacy learning benchmark is disabled: repair paired records, metrics and event features in stages 3-5 first. Run --mode physics for P0/P1 acceptance.')
    print("=" * 65)
    print("Running Full Benchmark: Baseline vs Fixed FF vs ILC vs ECLR")
    print("=" * 65)
    output_dir.mkdir(parents=True, exist_ok=True)
    runner = SimulationRunner()

    # 1. Training Phase
    print(f"\n[Phase 1] Collecting {n_train} training trials across approach states...")
    train_conds = sample_pre_event_conditions(n_train, seed=seed, test=False)
    X_train, Y_train = collect_eclr_dataset(runner, train_conds, mode='dock')

    print(f"[Phase 1] Fitting ECLR lifted model matrices A and G (samples: {len(X_train)})...")
    A, G = fit_eclr_matrices(X_train, Y_train, reg_lambda=1e-4)
    c_fixed = compute_fixed_feedforward(A, G, reg_lambda=1e-3, box_limit=0.8)
    print(f"[Phase 1] Fixed feedforward waveform c_fixed: {np.round(c_fixed, 4)}")

    # 2. ILC Pre-training on nominal condition
    print("\n[Phase 2] Pre-training Event-Aligned ILC on nominal approach...")
    ilc = EventAlignedILC(n_basis=4, learning_gain=0.03, max_limit=0.8)
    nom_cond = {'approach_offset': 0.0, 'q1_init_y': -0.6, 'q2_init_y': 0.6}
    for iter_idx in range(6):
        res = evaluate_trial(runner, 'fixed_ff', nom_cond, c_fixed=ilc.get_coefficients())
        if res['success']:
            # y error vector from pitch and rate
            y_sample = np.array(res['window_pitches'][:15], dtype=np.float64)
            if len(y_sample) == 15 and G.shape[0] >= 15:
                ilc.update(G[:15, :], y_sample)
    c_ilc = ilc.get_coefficients()
    print(f"[Phase 2] ILC converged coefficients: {np.round(c_ilc, 4)}")

    # 3. Held-out Evaluation Phase
    print(f"\n[Phase 3] Running held-out evaluation on {n_test} unseen conditions...")
    test_conds = sample_pre_event_conditions(n_test, seed=seed + 1000, test=True)

    methods = ['baseline', 'fixed_ff', 'ilc', 'eclr']
    records = []
    plot_trajectories = {m: [] for m in methods}

    for idx, cond in enumerate(test_conds):
        print(f"  Trial {idx+1:2d}/{n_test}: offset = {cond['approach_offset']*1000:+.1f} mm")
        for m in methods:
            coeff = c_fixed if m == 'fixed_ff' else (c_ilc if m == 'ilc' else None)
            res = evaluate_trial(runner, m, cond, c_fixed=coeff, A=A, G=G)
            if res['success']:
                rec = {
                    'trial': idx,
                    'method': m,
                    'offset_mm': cond['approach_offset'] * 1000.0,
                    'peak_pitch_deg': res['peak_pitch_deg'],
                    'peak_rate_deg_s': res['peak_rate_deg_s'],
                    'rms_pitch_deg': res['rms_pitch_deg'],
                    'settling_time_s': res['settling_time_s']
                }
                records.append(rec)
                if idx == 0:
                    plot_trajectories[m] = (res['window_times'], res['window_pitches'])

    # 4. Save CSV and JSON summary
    csv_path = output_dir / 'evaluation_trials.csv'
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    summary = {}
    for m in methods:
        m_recs = [r for r in records if r['method'] == m]
        summary[m] = {
            'peak_pitch_deg_mean': float(np.mean([r['peak_pitch_deg'] for r in m_recs])),
            'peak_pitch_deg_std': float(np.std([r['peak_pitch_deg'] for r in m_recs])),
            'peak_rate_deg_s_mean': float(np.mean([r['peak_rate_deg_s'] for r in m_recs])),
            'rms_pitch_deg_mean': float(np.mean([r['rms_pitch_deg'] for r in m_recs])),
            'settling_time_s_mean': float(np.mean([r['settling_time_s'] for r in m_recs]))
        }

    json_path = output_dir / 'summary.json'
    json_path.write_text(json.dumps({
        'metadata': '6-DoF MuJoCo Physics Evaluation with C++ Control Core',
        'n_train': n_train,
        'n_test': n_test,
        'summary': summary
    }, indent=2), encoding='utf-8')

    print("\n" + "=" * 65)
    print("EVALUATION SUMMARY ACROSS HELD-OUT CONDITIONS:")
    print("=" * 65)
    for m in methods:
        s = summary[m]
        print(f"  {m:10s} | Peak Pitch: {s['peak_pitch_deg_mean']:.3f} deg (+/- {s['peak_pitch_deg_std']:.3f}) | "
              f"Peak Rate: {s['peak_rate_deg_s_mean']:.2f} deg/s | RMS: {s['rms_pitch_deg_mean']:.3f} deg")

    # 5. Generate Figures
    generate_figures(output_dir, plot_trajectories, summary, records)
    print(f"\n[Done] Evaluation artifacts successfully generated in: {output_dir}\n")


def generate_figures(output_dir: Path, trajectories: dict, summary: dict, records: list):
    # Figure 1: Post-docking transient attitude comparison
    fig, ax = plt.subplots(figsize=(8.0, 4.5), dpi=160)
    colors = {'baseline': '#7f7f7f', 'fixed_ff': '#1f77b4', 'ilc': '#ff7f0e', 'eclr': '#2ca02c'}
    labels = {'baseline': 'B0: Bumpless Nominal', 'fixed_ff': 'B1: Fixed Feedforward',
              'ilc': 'B2: Event-Aligned ILC', 'eclr': 'B4: ECLR (Ours)'}

    for m in ('baseline', 'fixed_ff', 'ilc', 'eclr'):
        if trajectories[m]:
            t, p = trajectories[m]
            ax.plot(t, p, label=labels[m], color=colors[m], linewidth=2.0)

    ax.axhline(0.5, color='gray', linestyle='--', alpha=0.5, label='Tolerance (0.5 deg)')
    ax.axhline(-0.5, color='gray', linestyle='--', alpha=0.5)
    ax.set_title('Post-Docking Attitude Transient Comparison (MuJoCo 6-DoF)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time since capture confirmation (s)', fontsize=10)
    ax.set_ylabel('Pitch error (degrees)', fontsize=10)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', framealpha=0.9)
    fig.tight_layout()
    fig.savefig(output_dir / 'docking_transient_comparison.png')
    plt.close(fig)

    # Figure 2: Statistical bar chart with error bars
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.0, 4.2), dpi=160)
    methods = ['baseline', 'fixed_ff', 'ilc', 'eclr']
    x = np.arange(len(methods))
    m_labels = ['B0: Nominal', 'B1: Fixed FF', 'B2: ILC', 'B4: ECLR']

    peak_means = [summary[m]['peak_pitch_deg_mean'] for m in methods]
    peak_stds = [summary[m]['peak_pitch_deg_std'] for m in methods]
    rms_means = [summary[m]['rms_pitch_deg_mean'] for m in methods]

    bars1 = ax1.bar(x, peak_means, yerr=peak_stds, capsize=4, color=['#7f7f7f', '#1f77b4', '#ff7f0e', '#2ca02c'], alpha=0.85)
    ax1.set_xticks(x)
    ax1.set_xticklabels(m_labels, rotation=15)
    ax1.set_ylabel('Peak Pitch Error (deg)')
    ax1.set_title('Peak Post-Event Attitude Error')
    ax1.grid(True, linestyle=':', alpha=0.5, axis='y')

    bars2 = ax2.bar(x, rms_means, color=['#7f7f7f', '#1f77b4', '#ff7f0e', '#2ca02c'], alpha=0.85)
    ax2.set_xticks(x)
    ax2.set_xticklabels(m_labels, rotation=15)
    ax2.set_ylabel('RMS Pitch Error (deg)')
    ax2.set_title('RMS Post-Event Attitude Error')
    ax2.grid(True, linestyle=':', alpha=0.5, axis='y')

    fig.tight_layout()
    fig.savefig(output_dir / 'metrics_comparison.png')
    plt.close(fig)

    # Figure 3: Full scenario trajectory (Approach -> Dock -> Connected Flight -> Undock -> Recovery)
    runner = SimulationRunner()
    logs = runner.run_scenario(duration=7.5, dock_cmd_time=1.0, undock_cmd_time=5.0)
    t = [l['time'] for l in logs]
    y1 = [l['q1_y'] for l in logs]
    y2 = [l['q2_y'] for l in logs]
    z1 = [l['q1_z'] for l in logs]
    modes = [l['mode_name'] for l in logs]

    fig, (ax_pos, ax_alt) = plt.subplots(2, 1, figsize=(9.0, 5.0), sharex=True, dpi=160)
    ax_pos.plot(t, y1, label='Quad 1 Y-position (m)', color='#1f77b4', linewidth=1.8)
    ax_pos.plot(t, y2, label='Quad 2 Y-position (m)', color='#d62728', linewidth=1.8)
    ax_pos.axhline(-0.22, color='gray', linestyle=':', label='Docking Y-setpoints (+/-0.22m)')
    ax_pos.axhline(0.22, color='gray', linestyle=':')
    ax_pos.set_ylabel('Y Position (m)')
    ax_pos.set_title('Complete Flight Sequence: Approach, Docking, Connected Flight, Undocking', fontweight='bold')
    ax_pos.grid(True, linestyle=':', alpha=0.5)
    ax_pos.legend(loc='upper right')

    ax_alt.plot(t, z1, label='Altitude Z (m)', color='#2ca02c', linewidth=1.8)
    ax_alt.axhline(1.0, color='black', linestyle='--', label='Hover Altitude (1.0m)')
    ax_alt.set_ylabel('Altitude (m)')
    ax_alt.set_xlabel('Time (s)')
    ax_alt.grid(True, linestyle=':', alpha=0.5)
    ax_alt.legend(loc='lower right')

    fig.tight_layout()
    fig.savefig(output_dir / 'full_scenario_trajectory.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Dual Quadrotor Docking Simulation & Benchmark")
    parser.add_argument('--mode', choices=['smoke', 'physics', 'full'], default='smoke', help='P0/P1 execution; legacy full is gated pending stages 3-5')
    parser.add_argument('--output', type=Path, default=Path('results/mujoco_eval'), help='Output directory')
    parser.add_argument('--n-train', type=int, default=10, help='Number of training trials')
    parser.add_argument('--n-test', type=int, default=15, help='Number of held-out test trials')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    args = parser.parse_args()

    if args.mode == 'smoke':
        run_smoke_test()
    elif args.mode == 'physics':
        import subprocess
        subprocess.run([sys.executable,str(PROJECT_ROOT/'scripts'/'validate_stage2.py')],check=True)
    else:
        parser.error('Legacy full benchmark is disabled pending paired metrics and event-feature repairs (stages 3-5); use --mode physics.')


if __name__ == '__main__':
    main()

