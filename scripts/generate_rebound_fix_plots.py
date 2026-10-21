#!/usr/bin/env python3
"""Generate comparative diagnostic plots and data tables for docking rebound fix."""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "core" / "build"))

from mujoco_sim.python.sim_runner import SimulationRunner


def generate_plots():
    fig_dir = PROJECT_ROOT / "results" / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load Baseline Data
    baseline_csv = PROJECT_ROOT / "results" / "physics_validation" / "contact_capture_release" / "timeseries.csv"
    if not baseline_csv.exists():
        raise FileNotFoundError(f"Baseline CSV not found at {baseline_csv}")
    df_base = pd.read_csv(baseline_csv)

    # 2. Run Fixed Scenario
    runner = SimulationRunner()
    fixed_rows = runner.run_scenario(duration=12.0, dock_cmd_time=1.0, undock_cmd_time=6.0)
    df_fixed = pd.DataFrame(fixed_rows)

    # Key timeline events
    base_t_contact = 2.722
    base_t_dock = 4.906
    base_t_undock = 6.110

    fixed_t_contact = runner.first_contact_time
    fixed_t_dock = runner.dock_event_time
    fixed_t_undock = runner.release_event_time

    # -------------------------------------------------------------
    # Figure 1: Distance and Relative Velocity Comparison
    # -------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    t_min, t_max = 2.2, 5.5

    # Filter window
    m_base = (df_base["time"] >= t_min) & (df_base["time"] <= t_max)
    m_fixed = (df_fixed["time"] >= t_min) & (df_fixed["time"] <= t_max)

    t_b = df_base.loc[m_base, "time"].to_numpy()
    d_b = (df_base.loc[m_base, "rel_dist"] * 1000).to_numpy()  # mm
    vy_b = df_base.loc[m_base, "connector_relative_velocity_y"].to_numpy()

    t_f = df_fixed.loc[m_fixed, "time"].to_numpy()
    d_f = (df_fixed.loc[m_fixed, "rel_dist"] * 1000).to_numpy()  # mm
    vy_f = df_fixed.loc[m_fixed, "connector_relative_velocity_y"].to_numpy()

    # Top plot: Distance
    ax1.plot(t_b, d_b, "r--", linewidth=2.0, label="Baseline (No Capture Mechanism)")
    ax1.plot(t_f, d_f, "b-", linewidth=2.2, label="Fixed (With Physical Capture & Damping)")
    ax1.axhline(2.0, color="gray", linestyle=":", label="Lock Tolerance Threshold (2.0 mm)")
    
    # Annotate baseline peak
    ax1.annotate(f"Baseline Rebound: 107.61 mm\n(Separated for 2.11 s)",
                 xy=(3.552, 107.61), xytext=(3.7, 85),
                 arrowprops=dict(facecolor="red", shrink=0.05, width=1.5, headwidth=8),
                 fontsize=10, color="darkred", fontweight="bold")
    
    # Annotate fixed capture
    max_fixed_sep = np.max(df_fixed.loc[(df_fixed["time"] >= fixed_t_contact) & (df_fixed["time"] <= fixed_t_dock), "rel_dist"]) * 1000
    ax1.annotate(f"Fixed Rebound Suppressed: {max_fixed_sep:.2f} mm\n(Locked in 0.32 s)",
                 xy=(2.85, max_fixed_sep), xytext=(2.95, 30),
                 arrowprops=dict(facecolor="blue", shrink=0.05, width=1.5, headwidth=8),
                 fontsize=10, color="darkblue", fontweight="bold")

    ax1.set_ylabel("Connector Relative Separation (mm)", fontsize=11)
    ax1.set_title("Dual-Quadrotor MuJoCo Docking: Connector Distance & Velocity Comparison", fontsize=13, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=10)

    # Bottom plot: Relative Velocity
    ax2.plot(t_b, vy_b, "r--", linewidth=1.8, label="Baseline v_rel,y (m/s)")
    ax2.plot(t_f, vy_f, "b-", linewidth=2.0, label="Fixed v_rel,y (m/s)")
    ax2.axhline(0.0, color="black", linestyle="-", alpha=0.3)
    ax2.set_xlabel("Simulation Time (s)", fontsize=11)
    ax2.set_ylabel("Relative Normal Velocity (m/s)", fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower right", fontsize=10)

    for ax in (ax1, ax2):
        ax.axvline(base_t_contact, color="darkred", linestyle=":", alpha=0.6, label="First Contact (2.722s)")
        ax.axvline(fixed_t_dock, color="darkblue", linestyle="-.", alpha=0.7, label=f"Fixed Lock ({fixed_t_dock:.3f}s)")
        ax.axvline(base_t_dock, color="red", linestyle="-.", alpha=0.7, label=f"Baseline Lock ({base_t_dock:.3f}s)")

    fig.tight_layout()
    fig1_path = fig_dir / "rebound_distance_velocity_comparison.png"
    fig.savefig(fig1_path, dpi=200)
    plt.close(fig)
    print(f"Saved: {fig1_path}")

    # -------------------------------------------------------------
    # Figure 2: Forces Breakdown (Contact, Capture, Weld Load)
    # -------------------------------------------------------------
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    m = (df_fixed["time"] >= 2.5) & (df_fixed["time"] <= 6.5)
    t = df_fixed.loc[m, "time"].to_numpy()

    f_contact = df_fixed.loc[m, "contact_normal_force_N"].to_numpy()
    f_capture = df_fixed.loc[m, "capture_force_N"].to_numpy()
    weld_fx = df_fixed.loc[m, "weld_q1_force_world_N_x"].to_numpy()
    weld_fy = df_fixed.loc[m, "weld_q1_force_world_N_y"].to_numpy()
    weld_fz = df_fixed.loc[m, "weld_q1_force_world_N_z"].to_numpy()
    f_weld = np.sqrt(weld_fx**2 + weld_fy**2 + weld_fz**2)

    ax1.plot(t, f_contact, "tab:orange", linewidth=1.8, label="Physical Normal Contact Force (N)")
    ax1.set_ylabel("Contact Force (N)", fontsize=11)
    ax1.set_title("Fixed Docking Forces Breakdown: Impact Contact, Capture Mechanism, and Weld Load", fontsize=12, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)

    ax2.plot(t, f_capture, "tab:green", linewidth=2.0, label="Active Capture Mechanism Force (N)")
    ax2.set_ylabel("Capture Force (N)", fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", fontsize=9)

    ax3.plot(t, f_weld, "tab:purple", linewidth=1.8, label="Weld Constraint Force on Q1 (N)")
    ax3.set_ylabel("Weld Load (N)", fontsize=11)
    ax3.set_xlabel("Simulation Time (s)", fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=9)

    for ax in (ax1, ax2, ax3):
        ax.axvline(fixed_t_contact, color="orange", linestyle=":", alpha=0.7)
        ax.axvline(fixed_t_dock, color="green", linestyle="-.", alpha=0.8)
        ax.axvline(fixed_t_undock, color="purple", linestyle="--", alpha=0.8)

    ax1.text(fixed_t_contact + 0.02, ax1.get_ylim()[1] * 0.75, "CONTACT", color="darkorange", fontweight="bold", fontsize=9)
    ax2.text(fixed_t_dock + 0.02, ax2.get_ylim()[1] * 0.75, "LOCKED (Weld Active)", color="green", fontweight="bold", fontsize=9)
    ax3.text(fixed_t_undock + 0.02, ax3.get_ylim()[1] * 0.75, "UNDOCK", color="purple", fontweight="bold", fontsize=9)

    fig.tight_layout()
    fig2_path = fig_dir / "docking_forces_breakdown.png"
    fig.savefig(fig2_path, dpi=200)
    plt.close(fig)
    print(f"Saved: {fig2_path}")

    # -------------------------------------------------------------
    # Figure 3: Events Timeline Comparison
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 4.5))

    phases = [
        ("Approach", 1.0, 2.722, "gold"),
        ("Post-Contact Separation Rebound", 2.722, 4.870, "salmon"),
        ("Re-Contact & Dwell", 4.870, 4.906, "khaki"),
        ("Locked Aggregate Flight", 4.906, 6.110, "lightgreen"),
        ("Separation & Recovery", 6.110, 10.0, "skyblue"),
    ]
    for label, t0, t1, color in phases:
        ax.barh("Baseline", t1 - t0, left=t0, color=color, edgecolor="black", height=0.45)
        ax.text(t0 + (t1 - t0)/2, "Baseline", label if (t1 - t0) > 0.4 else "",
                ha="center", va="center", fontsize=8, fontweight="bold")

    phases_fixed = [
        ("Approach", 1.0, fixed_t_contact, "gold"),
        ("Capture & Dwell (0.32 s)", fixed_t_contact, fixed_t_dock, "lightgreen"),
        ("Locked Aggregate Flight", fixed_t_dock, fixed_t_undock, "palegreen"),
        ("Separation & Recovery", fixed_t_undock, 10.0, "skyblue"),
    ]
    for label, t0, t1, color in phases_fixed:
        ax.barh("Fixed", t1 - t0, left=t0, color=color, edgecolor="black", height=0.45)
        ax.text(t0 + (t1 - t0)/2, "Fixed", label if (t1 - t0) > 0.3 else "",
                ha="center", va="center", fontsize=8, fontweight="bold")

    ax.set_xlabel("Simulation Time (s)", fontsize=11)
    ax.set_title("Timeline Comparison: Baseline vs Fixed Capture Mechanism", fontsize=12, fontweight="bold")
    ax.set_xlim(0.8, 8.5)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig3_path = fig_dir / "events_timeline.png"
    fig.savefig(fig3_path, dpi=200)
    plt.close(fig)
    print(f"Saved: {fig3_path}")

    # -------------------------------------------------------------
    # Figure 4: Parameter Sensitivity & Approach Speed Sweep
    # -------------------------------------------------------------
    approach_durations = [1.5, 1.8, 2.0, 2.2, 2.5, 2.8, 3.2]
    app_speeds = []
    delays = []
    max_seps = []
    statuses = []

    for dur in approach_durations:
        r = SimulationRunner()
        opts = dict(approach_duration_s=dur)
        rows = r.run_scenario(duration=7.0, dock_cmd_time=1.0, research_options=opts)
        if r.first_contact_time is not None:
            # find contact velocity
            idx = np.abs(np.array([rw["time"] for rw in rows]) - r.first_contact_time).argmin()
            v_contact = abs(rows[idx]["connector_relative_velocity_y"])
            app_speeds.append(v_contact)
            
            if r.dock_event_time is not None:
                delays.append(r.dock_event_time - r.first_contact_time)
                # max sep
                c_rows = [rw for rw in rows if r.first_contact_time <= rw["time"] <= r.dock_event_time]
                max_seps.append(max(rw["rel_dist"] for rw in c_rows) * 1000)
                statuses.append("LOCKED")
            else:
                delays.append(np.nan)
                max_seps.append(np.nan)
                statuses.append("FAILED")
        else:
            app_speeds.append(np.nan)
            delays.append(np.nan)
            max_seps.append(np.nan)
            statuses.append("NO_CONTACT")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    ax1.plot(app_speeds, max_seps, "o-", color="tab:blue", linewidth=2.0, markersize=7)
    ax1.axhline(6.0, color="red", linestyle=":", label="Capture Travel Limit (6.0 mm)")
    ax1.set_xlabel("Impact Relative Normal Velocity (m/s)", fontsize=10)
    ax1.set_ylabel("Peak Post-Contact Separation (mm)", fontsize=10)
    ax1.set_title("Separation Rebound vs Impact Speed", fontsize=11, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left")

    ax2.plot(app_speeds, delays, "s-", color="tab:green", linewidth=2.0, markersize=7)
    ax2.set_xlabel("Impact Relative Normal Velocity (m/s)", fontsize=10)
    ax2.set_ylabel("Contact-to-Dock Time Delay (s)", fontsize=10)
    ax2.set_title("Lock Dwell Time vs Impact Speed", fontsize=11, fontweight="bold")
    ax2.grid(True, alpha=0.3)

    fig.suptitle("Robustness Across Approach Speeds (1.5s - 3.2s quintic profiles)", fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig4_path = fig_dir / "approach_speed_sensitivity_stats.png"
    fig.savefig(fig4_path, dpi=200)
    plt.close(fig)
    print(f"Saved: {fig4_path}")


if __name__ == "__main__":
    generate_plots()
