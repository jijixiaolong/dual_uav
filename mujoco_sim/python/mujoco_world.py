"""MuJoCo plant, SI units, world Z-up and body FLU (wxyz quaternions)."""
import sys
from pathlib import Path
import mujoco
import numpy as np
from .configuration import load_configuration

CORE_BUILD_DIR = Path(__file__).resolve().parents[2] / 'core' / 'build'
if str(CORE_BUILD_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_BUILD_DIR))
import docking_core_py as dcp


class MuJoCoWorld:
    """Two independent free bodies, eight site forces, motor lag and hard slew bounds."""

    def __init__(self, xml_path, dt=0.002, motor_tau=None):
        if dt <= 0:
            raise ValueError('Physics timestep must be positive')
        self.xml_path, self.dt = str(xml_path), float(dt)
        self.config = load_configuration()
        motor = self.config['quadrotor']['motor']
        self.motor_tau = motor['time_constant'] if motor_tau is None else motor_tau
        self.motor_slew = float(motor['slew_rate_limit'])
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)
        for i in (1, 2):
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f'quad{i}')
            joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'root_q{i}')
            setattr(self, f'q{i}_body_id', body)
            setattr(self, f'q{i}_qpos_adr', int(self.model.jnt_qposadr[joint]))
            setattr(self, f'q{i}_qvel_adr', int(self.model.jnt_dofadr[joint]))
        self.body_ids = [self.q1_body_id, self.q2_body_id]
        self.weld_eq_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, 'dock_weld')
        self._initial_eq_data = self.model.eq_data.copy()
        self.applied_thrusts = np.zeros(8)
        self.target_thrusts = np.zeros(8)
        self.requested_thrusts = np.zeros(8)
        self.force_providers = []
        self._managed_qforce = np.zeros(self.model.nv)
        self.connector_geoms = {mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,n)
                                for n in ('q1_conn_funnel','q2_conn_probe')}
        self.reset()

    def reset(self, q1_init_pos=(-0.6, 1.0), q2_init_pos=(0.6, 1.0)):
        mujoco.mj_resetData(self.model, self.data)
        self.model.eq_data[:] = self._initial_eq_data
        self._managed_qforce[:] = 0
        self.contact_impulse_est_Ns = 0.
        self.last_step_contact_normal_N = 0.
        for i, position in enumerate((q1_init_pos, q2_init_pos), 1):
            p = [0., *position] if len(position) == 2 else position
            adr = getattr(self, f'q{i}_qpos_adr')
            self.data.qpos[adr:adr + 7] = [*p, 1., 0., 0., 0.]
            hover = self.model.body_mass[getattr(self, f'q{i}_body_id')] * -self.model.opt.gravity[2] / 4
            self.applied_thrusts[4*(i-1):4*i] = hover
        self.target_thrusts[:] = self.applied_thrusts
        self.requested_thrusts[:] = self.applied_thrusts
        self.data.ctrl[:] = self.applied_thrusts
        self.set_weld_active(False)
        self.forward()

    def forward(self):
        """Refresh derived state AND forces at exactly data.time (mj_step leaves these stale)."""
        mujoco.mj_forward(self.model, self.data)

    def set_weld_active(self, active):
        self.data.eq_active[self.weld_eq_id] = bool(active)

    def is_weld_active(self):
        return bool(self.data.eq_active[self.weld_eq_id])

    def rotation(self, quad_idx):
        return self.data.xmat[getattr(self, f'q{quad_idx}_body_id')].reshape(3, 3).copy()

    def get_quad_state(self, quad_idx):
        if quad_idx not in (1, 2):
            raise ValueError('quad_idx must be 1 or 2')
        a, v = getattr(self, f'q{quad_idx}_qpos_adr'), getattr(self, f'q{quad_idx}_qvel_adr')
        st = dcp.RigidBodyState()
        st.pos = dcp.Vec3(*self.data.qpos[a:a+3])
        st.quat = dcp.Quat(*self.data.qpos[a+3:a+7])
        st.vel = dcp.Vec3(*self.data.qvel[v:v+3])
        st.omega = dcp.Vec3(*self.data.qvel[v+3:v+6])
        st.timestamp = self.time
        return st

    def step(self, motor_commands_8):
        requested = np.asarray(motor_commands_8, dtype=float)
        if requested.shape != (8,) or not np.isfinite(requested).all():
            raise ValueError('Expected eight finite motor thrust commands in N')
        self.requested_thrusts = requested.copy()
        desired = np.clip(requested, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        max_delta = self.motor_slew * self.dt
        self.target_thrusts += np.clip(desired - self.target_thrusts, -max_delta, max_delta)
        alpha = np.exp(-self.dt / self.motor_tau) if self.motor_tau > 0 else 0.
        lagged = alpha*self.applied_thrusts + (1-alpha)*self.target_thrusts
        self.applied_thrusts += np.clip(lagged-self.applied_thrusts, -max_delta, max_delta)
        self.data.ctrl[:] = self.applied_thrusts
        # Providers own only their additive component. Unrelated external forces survive.
        new_force = sum((p() for p in self.force_providers), np.zeros(self.model.nv))
        self.data.qfrc_applied += new_force - self._managed_qforce
        self._managed_qforce = new_force
        mujoco.mj_step(self.model, self.data)
        self.last_step_contact_normal_N = 0.
        for j in range(self.data.ncon):
            c = self.data.contact[j]
            if {int(c.geom1),int(c.geom2)} == self.connector_geoms:
                f = np.zeros(6)
                mujoco.mj_contactForce(self.model,self.data,j,f)
                self.last_step_contact_normal_N += abs(f[0])
        self.contact_impulse_est_Ns += self.last_step_contact_normal_N*self.dt
        self.forward()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError('Non-finite MuJoCo state')

    def system_quantities(self):
        """Momentum about world origin; kinetic+gravitational energy excludes soft contact energy."""
        masses = self.model.body_mass[self.body_ids]
        p, vel, omega, inertias = [], [], [], []
        for i, b in enumerate(self.body_ids, 1):
            a, v = getattr(self, f'q{i}_qpos_adr'), getattr(self, f'q{i}_qvel_adr')
            p.append(self.data.qpos[a:a+3].copy())
            vel.append(self.data.qvel[v:v+3].copy())
            omega.append(self.rotation(i) @ self.data.qvel[v+3:v+6])
            ri = self.data.ximat[b].reshape(3, 3)
            inertias.append(ri @ np.diag(self.model.body_inertia[b]) @ ri.T)
        p, vel, omega, inertias = map(np.asarray, (p, vel, omega, inertias))
        com = np.average(p, weights=masses, axis=0)
        vcom = np.average(vel, weights=masses, axis=0)
        linear = np.sum(masses[:, None]*vel, axis=0)
        angular = sum((I@w + np.cross(x, m*v) for I,w,x,m,v in zip(inertias,omega,p,masses,vel)), np.zeros(3))
        J = sum((I + m*(np.dot(r,r)*np.eye(3)-np.outer(r,r)) for I,m,r in zip(inertias,masses,p-com)), np.zeros((3,3)))
        hcom = angular - np.cross(com, linear)
        aggregate_omega_world = np.linalg.solve(J, hcom)
        kinetic = sum(.5*m*np.dot(v,v)+.5*w@I@w for m,v,w,I in zip(masses,vel,omega,inertias))
        potential = -sum(m*np.dot(self.model.opt.gravity,x) for m,x in zip(masses,p))
        return dict(com=com, com_velocity=vcom, aggregate_omega_world=aggregate_omega_world,
                    aggregate_inertia_world=J, linear_momentum=linear, angular_momentum=angular,
                    kinetic_energy_J=float(kinetic), potential_energy_J=float(potential),
                    mechanical_energy_J=float(kinetic+potential))

    @property
    def time(self):
        return float(self.data.time)
