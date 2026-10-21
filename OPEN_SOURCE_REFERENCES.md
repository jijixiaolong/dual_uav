# Repository and documentation references (checked 2026-10-21)

- ModQuad simulator (legacy ROS, original author project fork): https://github.com/ng4mf/modquad-simulator — GPL-3.0. Modules/dynamics helpful; not a verified modern latch/detach environment.
- MARS-FTCP: https://github.com/RuiHuangNUS/MARS-FTCP — C++/CoppeliaSim, look at control allocation and preexisting reconfiguration handling; no license surfaced at repository metadata; do not copy code without verification.
- MARS-Reconfig: https://github.com/RuiHuangNUS/MARS-Reconfig — CoppeliaSim modular reconfiguration reference.
- ProxFly: https://github.com/ruiqizhang99/ProxFly — MIT; simulator intentionally simplified by authors, not equivalent to full paper implementation.
- CrazySim: https://github.com/gtfactslab/CrazySim — GPL-3.0; Crazyflie firmware SITL; MuJoCo and Gazebo multi-agent support; not docking-latch out of the box.
- Crazyswarm2: https://github.com/IMRCLab/crazyswarm2 — ROS2 fleet and real-hardware control, not latch simulation.
- MuJoCo simulation state and weld toggling: https://mujoco.readthedocs.io/en/3.3.5/computation/ ; https://github.com/google-deepmind/mujoco/discussions/2323 — data.eq_active and weld constraint caveats.

Always verify exact Git commit, license, documented versions, build instructions before integrations and note dependencies. License caution particularly for GPL-3.0 and no-license repos.
