#!/usr/bin/env python3
"""Record uncontrolled MuJoCo collision and release invariants, with real artifacts.

    python3 scripts/validate_physical_events.py
    python3 scripts/validate_physical_events.py --output results/physics_validation/invariants

No learning, flight controller, injected impulse, or state reset is used at an event.
All cases have zero gravity and zero motor input. The spring's charging work is
explicitly recorded at preconnection; its release has a finite energy budget.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import traceback

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mujoco_sim.python.mujoco_world import MuJoCoWorld
from mujoco_sim.python.latch_manager import LatchManager

XML_PATH = ROOT / "mujoco_sim" / "assets" / "dual_quads.xml"
DEFAULT_OUTPUT = ROOT / "results" / "physics_validation" / "invariants"


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _write_json(path, value):
    path.write_text(json.dumps(_jsonable(value), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _make_world(dt, q1, q2):
    world = MuJoCoWorld(str(XML_PATH), dt=dt, motor_tau=0.0)
    world.model.opt.gravity[:] = 0.0
    world.reset(q1_init_pos=q1, q2_init_pos=q2)
    # Reset starts at weight support. Clear EVERY stage, so the rate limiter
    # cannot turn an intended zero-input conservation test into powered flight.
    world.target_thrusts[:] = 0.0
    world.applied_thrusts[:] = 0.0
    world.requested_thrusts[:] = 0.0
    world.data.ctrl[:] = 0.0
    world.data.qfrc_applied[:] = 0.0
    world.data.xfrc_applied[:] = 0.0
    world.forward()
    return world


def _vector(row, prefix, values, suffix="", axes=("x", "y", "z")):
    for axis, value in zip(axes, values):
        row[f"{prefix}_{axis}{suffix}"] = float(value)


def _net_wrench(world, generalized):
    force, moment = np.zeros(3), np.zeros(3)
    for i in (1, 2):
        a, v = getattr(world, f"q{i}_qpos_adr"), getattr(world, f"q{i}_qvel_adr")
        f = generalized[v:v + 3]
        force += f
        moment += np.cross(world.data.qpos[a:a + 3], f)
        moment += world.rotation(i) @ generalized[v + 3:v + 6]
    return force, moment


def _sample(world, latch, initial, phase):
    quantities = world.system_quantities()
    contacts = latch.contact_details()
    spring = latch.spring_force_generalized()
    weld = latch.weld_generalized_force()
    net_force, net_moment = _net_wrench(world, spring)
    row = dict(
        time_s=world.time, phase=phase, weld_active=int(world.is_weld_active()),
        contact_count=len(contacts),
        connector_contacts_json=json.dumps(_jsonable(contacts), separators=(",", ":")),
        contact_normal_N=float(sum(c["normal_force_N"] for c in contacts)),
        last_step_contact_normal_N=float(world.last_step_contact_normal_N),
        contact_normal_impulse_est_Ns=float(world.contact_impulse_est_Ns),
        spring_force_N=float(latch.preload_force),
        spring_energy_J=float(latch.spring_energy()),
        kinetic_energy_J=quantities["kinetic_energy_J"],
        gravitational_potential_energy_J=quantities["potential_energy_J"],
        mechanical_energy_J=quantities["mechanical_energy_J"],
        mechanical_plus_spring_energy_J=quantities["mechanical_energy_J"] + latch.spring_energy(),
        linear_momentum_residual_kg_m_s=float(np.linalg.norm(quantities["linear_momentum"] - initial["linear_momentum"])),
        angular_momentum_residual_kg_m2_s=float(np.linalg.norm(quantities["angular_momentum"] - initial["angular_momentum"])),
        spring_net_force_residual_N=float(np.linalg.norm(net_force)),
        spring_net_torque_residual_Nm=float(np.linalg.norm(net_moment)),
    )
    for key, suffix in (("linear_momentum", "_kg_m_s"), ("angular_momentum", "_kg_m2_s"),
                        ("com", "_m"), ("com_velocity", "_m_s")):
        _vector(row, key, quantities[key], suffix)
    contact_force = sum((c["force_on_q2_world"] for c in contacts), np.zeros(3))
    _vector(row, "contact_force_on_q2_world", contact_force, "_N")
    relative = latch.relative_state()
    _vector(row, "connector_relative_position", relative["position"], "_m")
    _vector(row, "connector_relative_velocity", relative["velocity"], "_m_s")
    row["connector_relative_attitude_rad"] = relative["angle_rad"]
    row["weld_q1_force_norm_N"] = float(np.linalg.norm(weld[world.q1_qvel_adr:world.q1_qvel_adr + 3]))
    row["weld_q1_torque_norm_Nm"] = float(np.linalg.norm(weld[world.q1_qvel_adr + 3:world.q1_qvel_adr + 6]))
    for i in (1, 2):
        a, v = getattr(world, f"q{i}_qpos_adr"), getattr(world, f"q{i}_qvel_adr")
        _vector(row, f"q{i}_position", world.data.qpos[a:a + 3], "_m")
        _vector(row, f"q{i}_quaternion", world.data.qpos[a + 3:a + 7], axes=("w", "x", "y", "z"))
        _vector(row, f"q{i}_velocity_world", world.data.qvel[v:v + 3], "_m_s")
        _vector(row, f"q{i}_omega_body", world.data.qvel[v + 3:v + 6], "_rad_s")
        for name, values in (("weld", weld), ("spring_current", spring), ("engine_applied", world.data.qfrc_applied)):
            _vector(row, f"q{i}_{name}_force_world", values[v:v + 3], "_N")
            _vector(row, f"q{i}_{name}_torque_body", values[v + 3:v + 6], "_Nm")
    eqdata = world.model.eq_data[world.weld_eq_id]
    for i, anchor in ((1, eqdata[3:6]), (2, eqdata[:3])):
        body_id = getattr(world, f"q{i}_body_id")
        _vector(row, f"q{i}_weld_anchor_world", world.data.xpos[body_id] + world.rotation(i) @ anchor, "_m")
    for i, anchor in enumerate(latch.spring_anchors(), 1):
        _vector(row, f"q{i}_spring_anchor_world", anchor, "_m")
    for i in range(8):
        row[f"motor_{i}_target_N"] = float(world.target_thrusts[i])
        row[f"motor_{i}_actual_N"] = float(world.applied_thrusts[i])
    return row


def _raw_snapshot(world, latch):
    return {"time_s": world.time, "weld_active": world.is_weld_active(),
            **latch.snapshot(), "weld_generalized_force": latch.weld_generalized_force().tolist(),
            "qfrc_applied": world.data.qfrc_applied.tolist(),
            "connector_contacts": _jsonable(latch.contact_details())}


def _release(world, latch, rows, initial):
    rows.append(_sample(world, latch, initial, "release_before"))
    before = _raw_snapshot(world, latch)
    released = latch.disengage_weld()
    after = _raw_snapshot(world, latch)
    rows.append(_sample(world, latch, initial, "release_after"))
    return {"time_s": world.time, "release_succeeded": released, "pre": before, "post": after}


def _advance(world, latch, rows, initial, duration, phase):
    for _ in range(int(round(duration / world.dt))):
        world.step(np.zeros(8))
        rows.append(_sample(world, latch, initial, phase))


def _check(checks, name, passed, measured, acceptance):
    checks[name] = {"status": "PASS" if bool(passed) else "FAIL",
                    "measured": _jsonable(measured), "acceptance": acceptance}


def _plot_case(path, rows, title, release_time):
    t = np.array([r["time_s"] for r in rows])
    column = lambda key: np.array([r[key] for r in rows])
    figure, axes = plt.subplots(4, 2, figsize=(12, 12), sharex=True)
    for i in (1, 2):
        axes[0, 0].plot(t, column(f"q{i}_position_y_m"), label=f"Q{i}")
        axes[0, 1].plot(t, column(f"q{i}_velocity_world_y_m_s"), label=f"Q{i}")
    axes[0, 0].set_ylabel("Y position (m)")
    axes[0, 1].set_ylabel("Y velocity (m/s)")
    for key, label in (("last_step_contact_normal_N", "Contact normal"),
                       ("weld_q1_force_norm_N", "Weld force on Q1"), ("spring_force_N", "Spring")):
        axes[1, 0].plot(t, column(key), label=label)
    axes[1, 0].set_ylabel("Force (N)")
    axes[1, 1].plot(t, column("contact_normal_impulse_est_Ns"), label="Integrated normal contact")
    axes[1, 1].set_ylabel("Contact impulse estimate (N s)")
    for key, label in (("mechanical_energy_J", "Mechanical"), ("spring_energy_J", "Spring storage"),
                       ("mechanical_plus_spring_energy_J", "Mechanical + spring")):
        axes[2, 0].plot(t, column(key), label=label)
    axes[2, 0].set_ylabel("Energy (J)")
    axes[2, 1].plot(t, np.max([[r[f"motor_{i}_actual_N"] for i in range(8)] for r in rows], axis=1), label="Maximum of eight motors")
    axes[2, 1].set_ylabel("Actual motor thrust (N)")
    axes[3, 0].plot(t, column("linear_momentum_residual_kg_m_s"), label="Linear momentum residual")
    axes[3, 0].set_ylabel("Momentum residual (kg m/s)")
    axes[3, 1].plot(t, column("angular_momentum_residual_kg_m2_s"), label="Angular momentum residual")
    axes[3, 1].set_ylabel("Angular residual (kg m²/s)")
    for axis in axes.flat:
        if release_time is not None:
            axis.axvline(release_time, color="black", linestyle=":", linewidth=1)
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("Physical time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def _run_case(name, output):
    release = None
    if name == "unforced_collision":
        config = dict(dt_s=0.001, duration_s=0.6, q1_position_m=[0, -0.24, 2], q2_position_m=[0, 0.24, 2],
                      q1_velocity_m_s=[0.07, 0.25, 0], q2_velocity_m_s=[0.07, -0.15, 0], zero_preload=True)
    elif name == "zero_preload_release":
        config = dict(dt_s=0.001, connected_duration_s=0.05, released_duration_s=0.3,
                      q1_position_m=[0, -0.2225, 2], q2_position_m=[0, 0.2225, 2],
                      q1_velocity_m_s=[0.1, 0.04, -0.02], q2_velocity_m_s=[0.1, 0.04, -0.02], zero_preload=True)
    else:
        config = dict(dt_s=0.00025, connected_duration_s=0.05, released_duration_s=0.6,
                      q1_position_m=[0, -0.22, 2], q2_position_m=[0, 0.22, 2],
                      q1_velocity_m_s=[0, 0, 0], q2_velocity_m_s=[0, 0, 0], zero_preload=False)
    config.update(gravity_m_s2=[0, 0, 0], motor_command_N=[0] * 8, initial_omega_body_rad_s=[0, 0, 0])
    world = _make_world(config["dt_s"], config["q1_position_m"], config["q2_position_m"])
    latch = LatchManager(world, zero_preload=config["zero_preload"])
    for i in (1, 2):
        v = getattr(world, f"q{i}_qvel_adr")
        world.data.qvel[v:v + 3] = config[f"q{i}_velocity_m_s"]
    world.forward()
    if name != "unforced_collision":
        latch.initialize_connected()
    initial = world.system_quantities()
    rows = [_sample(world, latch, initial, "initial")]
    if name == "unforced_collision":
        _advance(world, latch, rows, initial, config["duration_s"], "collision")
    else:
        _advance(world, latch, rows, initial, config["connected_duration_s"], "connected_loading")
        release = _release(world, latch, rows, initial)
        _advance(world, latch, rows, initial, config["released_duration_s"], "released")

    peak = lambda key: max(r[key] for r in rows)
    metrics = dict(max_linear_momentum_residual_kg_m_s=peak("linear_momentum_residual_kg_m_s"),
                   max_angular_momentum_residual_kg_m2_s=peak("angular_momentum_residual_kg_m2_s"),
                   peak_contact_force_N=peak("last_step_contact_normal_N"),
                   contact_impulse_est_Ns=rows[-1]["contact_normal_impulse_est_Ns"],
                   initial_mechanical_energy_J=rows[0]["mechanical_energy_J"],
                   final_mechanical_energy_J=rows[-1]["mechanical_energy_J"],
                   initial_spring_energy_J=rows[0]["spring_energy_J"],
                   final_spring_energy_J=rows[-1]["spring_energy_J"],
                   max_spring_net_force_N=peak("spring_net_force_residual_N"),
                   max_spring_net_torque_Nm=peak("spring_net_torque_residual_Nm"),
                   max_actual_motor_N=max(r[f"motor_{i}_actual_N"] for r in rows for i in range(8)))
    checks = {}
    _check(checks, "zero_motor_input", metrics["max_actual_motor_N"] == 0.0, metrics["max_actual_motor_N"], "Exactly 0 N on all eight motors")
    _check(checks, "linear_momentum", metrics["max_linear_momentum_residual_kg_m_s"] < 1e-8,
           metrics["max_linear_momentum_residual_kg_m_s"], "Maximum residual < 1e-8 kg m/s")
    _check(checks, "angular_momentum", metrics["max_angular_momentum_residual_kg_m2_s"] < 5e-6,
           metrics["max_angular_momentum_residual_kg_m2_s"], "Maximum residual < 5e-6 kg m²/s; finite-step tolerance")
    if name == "unforced_collision":
        _check(checks, "real_contact", metrics["peak_contact_force_N"] > 0.01 and metrics["contact_impulse_est_Ns"] > 1e-5,
               [metrics["peak_contact_force_N"], metrics["contact_impulse_est_Ns"]], "Contact peak > .01 N and accumulated normal impulse > 1e-5 N s")
        _check(checks, "collision_energy_dissipation", metrics["final_mechanical_energy_J"] < metrics["initial_mechanical_energy_J"] - 1e-5,
               metrics["initial_mechanical_energy_J"] - metrics["final_mechanical_energy_J"], "Dissipated kinetic energy > 1e-5 J")
        _check(checks, "independent_bodies", not any(r["weld_active"] for r in rows),
               sum(r["weld_active"] for r in rows), "Weld never active")
    else:
        pre, post = release["pre"], release["post"]
        metrics["instant_release_qpos_max_delta"] = float(np.max(np.abs(np.array(post["qpos"]) - pre["qpos"])))
        metrics["instant_release_qvel_max_delta"] = float(np.max(np.abs(np.array(post["qvel"]) - pre["qvel"])))
        metrics["instant_release_linear_momentum_delta_kg_m_s"] = float(np.linalg.norm(np.array(post["linear_momentum"]) - pre["linear_momentum"]))
        metrics["instant_release_angular_momentum_delta_kg_m2_s"] = float(np.linalg.norm(np.array(post["angular_momentum"]) - pre["angular_momentum"]))
        metrics["instant_release_mechanical_energy_delta_J"] = post["mechanical_energy_J"] - pre["mechanical_energy_J"]
        metrics["prerelease_weld_force_N"] = next(r["weld_q1_force_norm_N"] for r in rows if r["phase"] == "release_before")
        _check(checks, "release_state_continuity", metrics["instant_release_qpos_max_delta"] == 0 and metrics["instant_release_qvel_max_delta"] == 0,
               [metrics["instant_release_qpos_max_delta"], metrics["instant_release_qvel_max_delta"]], "qpos and qvel identical at the switch, before physical integration")
        _check(checks, "weld_released", release["release_succeeded"] and not world.is_weld_active(),
               {"release_succeeded": release["release_succeeded"], "weld_active_after_release": world.is_weld_active()},
               "Release succeeds and weld remains inactive")
        if config["zero_preload"]:
            velocity_change = float(np.max(np.abs(world.data.qvel - np.array(post["qvel"]))))
            energy_change = max(abs(r["mechanical_energy_J"] - rows[0]["mechanical_energy_J"]) for r in rows)
            metrics.update(max_qvel_change_after_release=velocity_change, max_mechanical_energy_change_J=energy_change)
            _check(checks, "no_phantom_release_acceleration", velocity_change < 1e-9, velocity_change, "All free velocities constant within 1e-9 SI")
            _check(checks, "zero_preload_energy", energy_change < 1e-10 and peak("spring_energy_J") == 0,
                   [energy_change, peak("spring_energy_J")], "Mechanical energy constant within 1e-10 J, no spring storage")
        else:
            after = [r for r in rows if r["phase"] in ("release_after", "released")]
            budget = pre["mechanical_energy_J"] + pre["spring_energy_J"]
            release_peak_energy = max(r["mechanical_plus_spring_energy_J"] for r in after)
            metrics.update(release_energy_budget_J=budget, peak_postrelease_energy_J=release_peak_energy,
                           spring_charging_work_J=latch.events[0]["spring_charging_work_J"])
            _check(checks, "measurable_prerelease_load", metrics["prerelease_weld_force_N"] > 0.1,
                   metrics["prerelease_weld_force_N"], "Weld force exceeds .1 N before release")
            _check(checks, "finite_charging_work", abs(metrics["spring_charging_work_J"] - .5*latch.spring_stiffness*latch.preload_deflection**2) < 1e-12,
                   metrics["spring_charging_work_J"], "Charging work equals configured initial elastic potential .5*k*x²")
            _check(checks, "release_energy_budget", release_peak_energy <= budget + 1e-5, release_peak_energy - budget,
                   "Mechanical + elastic energy never exceeds release budget by > 1e-5 J")
            _check(checks, "preload_has_observable_response", rows[-1]["kinetic_energy_J"] > 1e-5,
                   rows[-1]["kinetic_energy_J"], "Final kinetic energy > 1e-5 J from declared spring storage")
            _check(checks, "internal_spring_wrench", metrics["max_spring_net_force_N"] < 1e-10 and metrics["max_spring_net_torque_Nm"] < 1e-10,
                   [metrics["max_spring_net_force_N"], metrics["max_spring_net_torque_Nm"]], "Net spring force and world moment < 1e-10 SI")
            residual_force = float(np.max(np.abs(world.data.qfrc_applied)))
            _check(checks, "spring_unloads_and_force_stops", rows[-1]["spring_energy_J"] < 1e-10 and residual_force < 1e-10,
                   [rows[-1]["spring_energy_J"], residual_force], "Elastic storage and applied force vanish; no persistent release pulse")
            config.update(spring_stiffness_N_per_m=latch.spring_stiffness, preload_deflection_m=latch.preload_deflection,
                          spring_damping_Ns_per_m=latch.damping, spring_anchor_inset_m=0.004,
                          preload_source="Declared charging work at preconnection; no magnetic force law")

    csv_path, figure_path = output / f"{name}.csv", output / f"{name}.png"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _plot_case(figure_path, rows, name.replace("_", " "), release["time_s"] if release else None)
    result = dict(case=name, passed=all(c["status"] == "PASS" for c in checks.values()),
                  configuration=config, metrics=metrics, checks=checks, physical_steps=int(round(world.time / world.dt)),
                  csv_samples=len(rows), release_snapshot=release, latch_event_snapshots=latch.events,
                  artifacts={"csv": str(csv_path.resolve()), "figure": str(figure_path.resolve())})
    _write_json(output / f"{name}.json", result)
    return result


def run_validation(output_dir=DEFAULT_OUTPUT):
    """Run all three cases, retaining failed cases and returning a JSON-safe summary."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    cases = []
    for name in ("unforced_collision", "zero_preload_release", "spring_preload_release"):
        try:
            case = _run_case(name, output)
        except Exception as error:
            case = dict(case=name, passed=False, error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
            _write_json(output / f"{name}.json", case)
        cases.append(case)
    result = dict(schema_version=1, passed=all(case["passed"] for case in cases), mujoco_version=mujoco.__version__,
                  model_path=str(XML_PATH), units="SI; wxyz quaternions; world Z-up, body FLU",
                  notes=["No flight controller or learning compensation; all eight motor inputs exactly zero.",
                         "Preconnection at t=0 is an explicit initial condition, not a contact capture shortcut.",
                         "Contact impulse is the sum of per-step normal contact force * physics dt, a solver estimate.",
                         "Mechanical energy excludes soft contact elastic energy; collision acceptance compares settled endpoints.",
                         "Current spring force is evaluated at row time; engine_applied force was held during the preceding step.",
                         "Weld force/torque columns are extracted from equality rows only, excluding contact loads.",
                         "Angular momentum uses a stated finite-step tolerance; residual time series are preserved."], cases=cases)
    _write_json(output / "summary.json", result)
    return _jsonable(result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    summary = run_validation(args.output)
    for case in summary["cases"]:
        print(f"{'PASS' if case['passed'] else 'FAIL'} {case['case']}")
        for name, check in case.get("checks", {}).items():
            print(f"  {check['status']} {name}: {check['measured']}")
        if "error" in case:
            print(f"  {case['error']}")
    print(f"Artifacts: {args.output.resolve()}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
