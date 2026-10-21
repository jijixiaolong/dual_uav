# CODEX TASK — MuJoCo-only simulation, future PX4 + ROS 2 hardware deployment

## User-approved system architecture (supersedes older CODEX_TASK.md / EXPERIMENT_PLAN_CN.md where different)

- **Simulation**: MuJoCo, exclusively, for both quadrotor rigid-body / rotor physics AND docking/contact / connection / release. No Gazebo requirement, no Gazebo integration, no CoppeliaSim integration.
- **Hardware**: two independent PX4 flight controllers, one in each quadrotor; algorithms that must execute at deterministic high rate should ultimately run as PX4 firmware C++ modules, NOT as latency-sensitive ROS 2 laptop loops.
- **ROS 2**: Vicon/OptiTrack pose receiver and frame conversion, cross-vehicle relative state/supervisory coordination, logging and experiment orchestration, distribution of validated learned model parameters. PX4 <-> ROS2 bridge requires a specific pinned PX4 firmware and matching `px4_msgs` version; do not assume unverified names or namespaces.
- **Source compatibility**: Put reusable event classifier, nominal mode-aware correction, bounded feedforward, small-matrix predictor and safety projection in a small deterministic platform-independent C++ library, compiled both for desktop MuJoCo testing and PX4 deployment. No MuJoCo, ROS 2, or uORB includes inside the core.
- **Learning**: model fitting / QP optimization may initially run offline in Python after each simulated dock/undock trial; export compact, bounded event-conditioned correction coefficients for online execution. Do not force PPO or any GPU training. If the paper later requires truly online model updates, stage and justify them separately.

## Research objective
Two independently hovering quadrotors dock horizontally, form a connected pair, hover/track a mild trajectory, mechanically undock, and regain independent hover. Evaluate whether learning across repetitions of the **capture and release events** lowers short-horizon peak attitude error, angular-rate peaks, COM / individual position error, settling time, and actuator command slew compared to mature bumpless hybrid geometric control, fixed feedforward, and traditional event-aligned ILC. Improvement is a hypothesis to test, not an assumption.

## Suggested directory structure (you may improve it)

```
dual_quad_docking/
  core/                     # pure C++17; deterministic, no ROS/px4/mujoco symbols
    include/docking/{state,controller,events,predictor,safety,allocation}.hpp
    src/*.cpp
    tests/*.cpp
  mujoco_sim/
    assets/two_quads.xml    # two freejoint bodies, 8 rotors and connector contacts
    python/{world,actuator,events,bridge,runner,logger}.py
    bindings/              # pybind11 wrapper or C++ executable to invoke exactly core logic
  offline_learning/
    fit_event_model.py
    select_coefficients.py
    evaluation.py
  px4_adapter/              # separate later deliverable: out-of-tree module or validated control integration
    README_PORTING.md
    examples/               # code skeletons only until PX4 version + board decided
  ros2_adapter/             # separate later deliverable: timestamped interface declarations/skeletons
    README_PORTING.md
  configs/{quad_nominal,event_windows,learning,experiments}.yaml
  tests/                    # physical invariants + reproducibility
  experiments/              # synthetic and MuJoCo data separate
  results/                  # data and figures with complete provenance
  README.md
```

## Implementation milestones

### Milestone M0 — reference audit / experimental spec
- Inspect and link original repositories, do not clone + copy GPL or unlicensed code into product without license check.
- Key resources: `google-deepmind/mujoco` docs (`mjData.eq_active`); `ng4mf/modquad-simulator` for modular-aircraft physical assumptions; `RuiHuangNUS/MARS-FTCP` (reference only if unclear license); `RuiqiZhang99/ProxFly` (optional residual-learning reference); `PX4/PX4-Autopilot` out-of-tree modules/uORB guides; `PX4/px4_msgs` matching firmware version.
- Record explicit geometric / world / ROS / PX4 coordinate conventions: MuJoCo world Z-up, PX4 navigation NED/FRD vs ROS ENU/FLU as applicable. Unit test conversions and rotation/torque signs.

### Milestone M1 — MuJoCo physically valid two-quad flight
- Author MuJoCo MJCF with two distinct **freejoint** 6-DoF rigid bodies, each with its own inertial parameters and four rotor actuators, realistic motor thrust/drag torque map and first-order motor response (with separately configurable lag), saturations, sample/hold control updates.
- Implement geometric position and attitude control for each independently; test single and dual hover, small tracking trajectories. Use exact *same core C++ control logic* through desktop bindings; MuJoCo is a pure plant / sensors backend.
- Add separated and connected aggregate-wrench allocation. Both must respect individual motor thrust bounds and slew limits; allocation must reconstruct achievable full force and moment, with geometrical lever arms. Initialize aggregate inertia using parallel-axis theorem and verify against physical model.
- Physics tests: hover motor thrust matches gravity, no controller sign inversion, free fall, finite quaternion and velocities, conservation for no force, reproducible fixed-step integration, saturation never violated.

### Milestone M2 — dock/undock physical transitions, learning still disabled
- Build a light passive **connection interface**, two connector sites/geometry in MuJoCo, with contact enabled where intended and collision exclusions only where justified. Add a registered connection detector; proximity alone is NOT a successful latch.
- Add predeclared `<equality><weld ... active="false"/></equality>` between bodies and enable via `data.eq_active[weld_id]=1` only after connector alignment, relative velocity and latch conditions pass. Ensure `relpose` matches intended pose and calculate/detect massive solver impulses from mismatches; avoid reattaching distant bodies.
- Before weld: actual collision/contact solver; optional magnetic attraction must be an explicitly validated custom force-vs-gap model, distinct from weld itself. Do not say weld simulates magnet force.
- For release, enable a release transition guard, disable weld; zero-preload ideal undocking has no arbitrarily injected velocity/angle jump. Optional connector preload should have measurable force/elastic storage and physically consistent force change, not stochastic attitude teleportation.
- Modes: FREE, APPROACH, ALIGN, CAPTURE_PENDING, CONNECTED, RELEASE_PREP, SEPARATING, RECOVERY, ABORT. Add guards, timeouts, two-flight-controller heartbeat, lock/unlock event acknowledgment and sane fallback.
- Tests: zero initial relative velocity capture, momentum/energy accounting for impact, zero-preload continuity on release, no unphysical instantaneous pose jump, plausible constraint forces, re-docking after sufficient separation.
- Deliver independent and coupled flight *without learning* plus dock+undock short recorded successful scenario.

### Milestone M3 — event-specific learning and baselines
- B0: verified geometric controllers + aggregate feasible allocation + bumpless transfer; avoid an unfair unsafe baseline.
- B1: fixed precomputed event-specific feedforward from training data.
- B2: conventional event-aligned ILC (repeatable same initial-state cases), input limits and regularized stable update.
- B3: optional classical disturbance observer / adaptive baseline where adequately implemented.
- B4: **ECLR** event-conditioned lifted residual response predictor `y = A_e phi(xi_pre) + G_e c + eps` with separate DOCK and UNDOCK identification. Inputs `xi_pre` must be available/estimable on future hardware; never train on privileged MuJoCo true contact impulse unless marked oracle-only.
- Offline ridge regression plus low-dimensional constrained quadratic optimization; export small coefficient vectors; C++ core evaluates a **time-local bounded smooth feedforward**. If an optimizer remains offboard, freeze coefficients within each event and enforce hard allowed correction + slew constraints onboard.
- Distinguish learning timestamps and physical mode confirmation; update learned policy only between trials initially. Actuator changes should not be discontinuous when disabling compensation.

### Milestone M4 — scientific experiments & evaluation
- Dock/undock separately: identical repeated event; variable relative approach state; variation in motor time constants, masses/inertia and connector properties; transition time delay; held-out conditions, held-out random seeds.
- For true ILC vs ECLR generalization, train on an explicit train split and test on held-out event states; no retraining in test unless experiment says adaptation; use equal actuator limits, same nominal controllers, same event detector and timing policy.
- Log trial_id/seed/config hash; MuJoCo true vs noisy sensed state; relative connector error and speed; PX4-like local state; controls before/after rate/limit projection; motor thrust; latch state; constraint forces; estimated pre-event feature; learned output; command limits; failure/abort; time-stamps and model commit.
- Metrics: peak post-event SO(3) geodesic error; angular-rate peaks; peak connected COM error / post-release individual errors; settling time under predeclared criterion; input slew; saturation; success and abort fraction. Report distributions, CIs and per-event timing plots; synthetic surrogate results must not be represented as real MuJoCo data.

### Milestone M5 — PX4 deployment design, NO real flight unless authorized and hardware details known
- Target **PX4 onboard execution**: create a self-contained `px4_adapter/README_PORTING.md` and a buildable example out-of-tree module once board and PX4 branch are pinned. Preserve PX4 existing certified/stable low-level attitude/rate loop where practical. Define where correction is added (attitude/rate/torque/allocator): do not silently command raw motors or run two conflicting control authorities.
- For coupled flight, one shared intended wrench should be distributed into two locally feasible command vectors; determine how authority, time sync and input exchange remain consistent between two separate PX4 instances. ROS2 supplies low-rate higher-level coordination and external state, NOT an unbounded low-latency stabilization loop.
- If ROS2/network/mocap loses freshness, both aircraft should select a prevalidated fallback appropriate to the *physical latch state*. Do not assume independent free-flight fallback while still latched; don't disarm a single motor set when physically connected. Include time synchronization and staleness tests.
- Runtime ONBOARD: event-gated coefficients + control correction, hard limits, safety fallback, deterministic timing. Runtime OFFBOARD: training, historical data regression, large QP if not bounded, plotting and logging. Optional online learning only after demonstrated safety and timing bounds.
- PX4 docs: https://docs.px4.io/main/en/advanced/out_of_tree_modules ; https://docs.px4.io/main/en/middleware/uorb ; https://docs.px4.io/main/en/ros/ros2_comm ; https://docs.px4.io/main/en/tutorials/motion-capture . Check actual release docs and versioned topic definitions.

## Acceptance criteria for first Codex iteration
1. MuJoCo-only environment with 2 free-moving quadrotors, 8 motors, a geometric hover controller and physical validated connected hover (no learning required).
2. Deterministic headless smoke run and test suite, configuration file for nominal physical constants, documented screenshots or rendered video optional.
3. C++17 `core` compiles outside PX4, at least hover/allocation/event-gate routines callable by simulator (bindings or clean subprocess); do not duplicate independent Python control logic that diverges from deployment.
4. Clear declaration of whether M2 complete or not; don't claim contact/undock simulation until successful tests.
5. Subsequent changes strictly sequential: physical connection -> baselines -> ECLR -> PX4 adapter.

## Non-goals
- Gazebo, CoppeliaSim, mandatory ROS 2/PX4 SITL for algorithm proof of concept.
- End-to-end PPO/SAC as core baseline; raw PWM exploratory RL.
- Claiming MuJoCo intrinsically guarantees real-life contact fidelity.
- Claiming globally stable closed-loop hybrid system without a proof.
- Claiming implementation on real PX4 hardware without compiling/testing against a pinned PX4 target.
