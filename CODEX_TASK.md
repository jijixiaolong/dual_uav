# Codex development task — Dual-quadrotor docking / undocking simulation

## Scientific goal
Build a reproducible research simulation for **two horizontally docking and later undocking quadrotors**. The core claim to test, NOT presume: a state-conditioned learned, bounded, event-local feedforward correction reduces post-docking/post-undocking transients relative to (1) verified mode-aware geometric control with rate-limited allocation, (2) a fixed feedforward waveform, (3) classic event-aligned ILC, and (4) a conventional feedback/observer compensation baseline. Use the manuscript equations in `Manuscript_IEEE.tex` if available. Do not write a paper claiming improvement before data exists.

## Checked references and license constraints
1. `https://github.com/ng4mf/modquad-simulator` — original modular quadrotor simulator fork, legacy ROS/crazyflie stack, GPL-3.0. Inspect before reuse; likely not a drop-in modern contact simulator.
2. `https://github.com/RuiHuangNUS/MARS-FTCP` — C++ / CoppeliaSim-focused modular control/trajectory open repository; no repository license was found in metadata, so treat code as reference and don't copy code without license clarification.
3. `https://github.com/RuiHuangNUS/MARS-Reconfig` — modular self-reconfiguration, CoppeliaSim examples; inspect as reference.
4. `https://github.com/ruiqizhang99/ProxFly` — ICRA2025 residual RL, MIT; authors warn published simulator is simplified. OPTIONAL only for near-flight perturbations, not the main method.
5. `https://github.com/gtfactslab/CrazySim` — Crazyflie firmware SITL with MuJoCo and Gazebo; GPL-3.0, possible integration if target platform is Crazyflie.
6. `https://github.com/IMRCLab/crazyswarm2` — ROS2 multi-Crazyflie stack; useful for migration, NOT a magnetic connector physics model.
7. MuJoCo docs `https://mujoco.readthedocs.io/en/3.3.5/computation/`: `mjData.eq_active` can toggle a configured `<weld>` constraint at runtime. IMPORTANT: weld `relpose` is defined at compile-time unless explicitly managed. Check constraint consistency at capture and avoid accidental impulse injection from misaligned weld.

## Repo already supplied
- `src/event_surrogate.py`, `src/run_demo.py`, `tests/test_surrogate.py`: a RUNNABLE SYNTHETIC surrogate for event response identification. It is not 6DoF, no real contact, no mechanical docking, and cannot be used as publication evidence. Keep it in a clearly separated `surrogate/` or `experiments/toy_validation/` module.

## Required implementation phases (do not skip tests)
### Phase 1: physically consistent Python 6DoF hybrid model
- Two independent quadrotor rigid bodies on SO(3), using positions, velocities, rotation matrices or unit quaternions, angular velocities, realistic mass/inertia, rotor thrust & actuator lag.
- Stable independent SE(3) geometric control with each module's mass/inertia.
- A *rigid assembled configuration* with COM and parallel-axis inertia, a physically realizable aggregate wrench/rotor-thrust allocator, limits and slew constraints.
- Hybrid event states `SEPARATE`, `CAPTURE_CANDIDATE`, `CONNECTED`, `RELEASE_PREP`, `SEPARATING`, `RECOVERY`, `ABORT`. Supervisory logic must not treat proximity alone as confirmed docking.
- Docking impact mapping: conserve total linear and angular momentum in an ideal isolated instantaneous capture, impose connected velocity compatibility; kinetic energy may decrease. If a deliberate magnetic/structural impulse model is added, articulate the physical source.
- Undocking: for ideal stress-free release, keep individual bodies' velocities continuous from the rigid-body kinematics; NO gratuitous velocity jump. If nonzero preload/release impulse is modeled, apply opposite impulses at coincident interface attachment points and check total momentum conservation.
- Unit tests: stationary hover; free flight rotor command limits; coordinate-frame conventions; momentum conservation at capture & release; no phantom release impulse when preload=0; positive inertia; conservation in no-force test; no rate-constraint violation.

### Phase 2: event-wise baselines and learned controller
- Full baseline: mode-consistent geometric control plus feasible allocation plus bumpless supervisor. THIS is B0, not the unsafe 'switch PID' baseline.
- B1: fixed hand-tuned/best-offline short feedforward at both events.
- B2: conventional event-aligned ILC: fixed timing waveforms, verified input-to-error learning operator and constraints.
- B3: online disturbance observer or established adaptive control, held to same actuation limits (when sensible).
- B4: event-conditioned lifted residual learner (ECLR), separate dock and undock models, supervised fit of `y = A phi(xi_pre) + G c + epsilon`, ridge/recursive regression, optimize short feedforward coefficients via small constrained QP. Only learn on recorded runs and use held-out features for generalization tests. For fair tests, no privileged simulator state may be fed to learner at eval time.
- Gate learned compensation within bounded event window; blend and rate limit command; saturation fallback; use event timestamps *after* state supervisor confirms event.
- Important: if control effectiveness varies strongly at capture, experiment with a feedback residual policy as separate baseline, but DO NOT make PPO a prerequisite.

### Phase 3: MuJoCo contact and attachment
- First reproduce hovering two models with validated actuators and masses. Then add lightweight connector sites and passive capture region. Use true contact physics where possible and a predeclared weld equality toggled through `data.eq_active` only when the connector geometry/velocity conditions pass. Check that weld reference pose matches engaged geometry; read MuJoCo equality constraint docs and examples.
- For release, disable the weld; verify no spurious impulse from the solver. To represent elastic preload, use a *declared physical spring/connector model* (not random attitude jumps), and measure actual stored energy.
- IMPORTANT: MuJoCo weld constraints alone do not simulate magnets or locking hardware. Magnetic force-distance and latch-success models require separate logic and validation. Label all model assumptions.
- Provide headless CI option and optional video rendering. Do not rely on GPU.

### Phase 4: experiments and reporting
- Each of dock and undock: (i) nominal repeated initial conditions, (ii) random train initial states, (iii) held-out event states, (iv) varying inertia / motor lag / latch stiffness, (v) altered event timing. Controlled common-random-number comparisons among controllers for fair outcomes.
- Use repeated seeds (e.g., >= 10 independently sampled setups in early dev; set final replicate count with confidence intervals and power consideration). No invented performance guarantees.
- Log to CSV or parquet: mode, event confirmation timestamp, true & sensed poses, rel pose and velocity at connector, rotor thrust commands/applied thrust, aggregate wrench, constraints/contact impulses, learning features, learned coefficients, reference, error, saturation, abort, seed and source commit SHA.
- Metrics: post-event peak geodesic attitude error and angular rate; peak COM error for dock; worst individual position error after undock; settling time with stated threshold and dwell window; command slew / saturation / cost; capture/release success & failure/abort count; energy or work if meaningful.
- Figures: representative event-aligned timeseries; distribution across seeds; train-count learning curves; ILC vs event-conditioned held-out; critical ablations (no pre-state features, no correction, no bumpless handoff, no allocator limits).
- Include negative outcome analysis: if conventional bumpless switching is already near-optimal, report it instead of fabricating learning benefit.

## Expected package and acceptance checks
- Python package with `pyproject.toml`, typed modules `dynamics`, `controllers`, `hybrid_events`, `allocation`, `learning`, `evaluation`, `mujoco_backend`, and reusable YAML configs.
- `python -m pytest -q` passes; `python -m ... smoke_test --headless` exercises two models; one deterministic fully reproducible docking+undocking cycle exports data/figures.
- `README.md` documents environment pinning, model assumptions, setup and commands; create `.gitignore` and license review; DO NOT silently vendor GPL/no-license code.
- Add a detailed `validation_report.md` distinguishing physical consistency tests, toy benchmark, actual MuJoCo simulator results, limitations and failures.
- Don't claim full SE(3) implementation or real contact physics if only the supplied surrogate script is working.

## Definition of Done for Codex first PR
Physical 6DoF dynamics + geometric independent & connected hover + validated event maps + deterministic smoke tests; no learning needed in PR1. PR2 implements learned correction and evaluation. PR3 adds MuJoCo constraints and release. Each PR should be testable independently.
