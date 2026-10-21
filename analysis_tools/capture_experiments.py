"""Unpowered native-rigid-body capture experiments, independent of flight control."""
from pathlib import Path
import numpy as np
import mujoco
from mujoco_sim.python.mujoco_world import MuJoCoWorld
from mujoco_sim.python.latch_manager import LatchManager
XML=Path(__file__).resolve().parents[1]/'mujoco_sim/assets/dual_quads.xml'

def _totals(world):
    """World-frame momentum and kinetic energy from the two native rigid bodies."""
    momentum = np.zeros(3)
    angular = np.zeros(3)
    kinetic = 0.0
    for body_id, qpos, qvel in (
        (world.q1_body_id, world.q1_qpos_adr, world.q1_qvel_adr),
        (world.q2_body_id, world.q2_qpos_adr, world.q2_qvel_adr),
    ):
        mass = float(world.model.body_mass[body_id])
        velocity = world.data.qvel[qvel:qvel + 3]
        omega_body = world.data.qvel[qvel + 3:qvel + 6]
        rotation = world.data.xmat[body_id].reshape(3, 3)
        inertia_diag = world.model.body_inertia[body_id]
        linear = mass * velocity
        momentum += linear
        angular += np.cross(world.data.qpos[qpos:qpos + 3], linear)
        angular += rotation @ (inertia_diag * omega_body)
        kinetic += 0.5 * mass * float(velocity @ velocity)
        kinetic += 0.5 * float(omega_body @ (inertia_diag * omega_body))
    return momentum, angular, kinetic

def fixture(dt=.0005, offset=(0,0,0)):
    w = MuJoCoWorld(XML,dt=dt,motor_tau=0.)
    w.model.opt.gravity[:] = 0.
    w.reset(q1_init_pos=(0,-.2199,2),q2_init_pos=np.array([0,.2199,2])+offset)
    l = LatchManager(w)
    w.forward()
    l.update_capture_state()
    return w,l

def capture_experiment(dt=.0005, damping_only=False):
    w,l = fixture(dt,offset=(.001,0,.0005))
    assert l.is_captured and l.contact_details()
    if damping_only:
        l.capture_stiffness = 0.
    # Isolate the internal tensile element AFTER genuine contact at t=0.
    # Removing external contact is a laboratory fixture, never a flight shortcut.
    w.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    w.data.qvel[:] = [.03,-.025,.01,.03,.02,-.02,.03,.025,.01,-.03,.01,.02]
    w.forward()
    initial = (*_totals(w),l.capture_potential(),l.capture_reservoir_energy_J)
    initial_u = l.capture_potential()
    l.capture_transferred_energy_J = initial_u
    series = []
    for _ in range(round(.12/dt)):
        w.step(np.zeros(8))
        p,h,k = _totals(w)
        series.append(dict(time=w.time,linear_error=float(np.linalg.norm(p-initial[0])),
                           angular_error=float(np.linalg.norm(h-initial[1])),kinetic_J=k,
                           capture_potential_J=l.capture_potential(),
                           reservoir_J=l.capture_reservoir_energy_J,
                           dissipated_J=l.capture_dissipated_energy_J,
                           discarded_J=l.capture_discarded_energy_J,
                           transferred_J=l.capture_transferred_energy_J,
                           power_W=l.last_capture_diagnostics['damping_power_W']))
    return w,l,initial,series
