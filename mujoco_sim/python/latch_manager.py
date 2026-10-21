"""Contact-confirmed passive capture and finite-energy release, no magnetic force law."""
import mujoco
import numpy as np


class LatchManager:
    def __init__(self, world, zero_preload=True, preload_force=None):
        self.world, self.model, self.data = world, world.model, world.data
        self.params = world.config['docking']['latch_criteria']
        spring = world.config['docking']['release']['elastic_preload']
        capture_cfg = world.config['docking'].get('capture_mechanism', {})
        self.capture_enabled = bool(capture_cfg.get('enabled', True))
        self.capture_distance = float(capture_cfg.get('capture_distance_m', 0.040))
        self.capture_angle = float(capture_cfg.get('capture_angle_rad', 0.15))
        self.max_capture_force = float(capture_cfg.get('max_attraction_force_N', 6.0))
        self.capture_stiffness = float(capture_cfg.get('stiffness_N_per_m', 140.0))
        self.capture_damping = float(capture_cfg.get('damping_Ns_per_m', 10.0))
        self.capture_ang_damping = float(capture_cfg.get('angular_damping_Nms_per_rad', 0.04))
        self.filter_contact_on_lock = bool(capture_cfg.get('filter_contact_on_lock', True))
        self.contact_hold_duration = float(capture_cfg.get('contact_hold_duration_s', 0.040))
        self.zero_preload = zero_preload
        self.spring_stiffness = float(spring['spring_stiffness_N_per_m'])
        self.preload_deflection = float(spring['preload_deflection_m'])
        if preload_force is not None:
            self.preload_deflection = float(preload_force)/self.spring_stiffness
        self.damping = float(spring['damping_Ns_per_m'])
        self.q1_conn_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'q1_conn_funnel')
        self.q2_conn_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'q2_conn_probe')
        self.sites = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'q{i}_conn_site') for i in (1,2)]
        self.world.force_providers.append(self.spring_force_generalized)
        self.world.force_providers.append(self.capture_force_generalized)
        self.reset()

    def reset(self):
        self.latched = False
        self.last_contact_force = 0.
        self.last_capture_force = 0.
        self.is_captured = False
        self.has_contacted = False
        self.last_contact_time = -1.0
        self.disengaged = False
        self.spring_armed = False
        self.spring_rest_length = 0.
        self.preload_force = 0.
        self.events = []
        self.set_connector_collision_active(True)
        self.world.set_weld_active(False)

    def set_connector_collision_active(self, active: bool):
        val = 1 if active else 0
        self.model.geom_contype[self.q2_conn_geom] = val
        self.model.geom_conaffinity[self.q2_conn_geom] = val

    def contact_details(self):
        contacts = []
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            if {int(con.geom1), int(con.geom2)} == {self.q1_conn_geom, self.q2_conn_geom} and con.efc_address >= 0:
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                # Contact frame axes are ROWS; force is on geom2.
                force_world = con.frame.reshape(3,3).T @ force[:3]
                if con.geom2 != self.q2_conn_geom:
                    force_world *= -1
                contacts.append(dict(position_world=con.pos.copy(), normal_force_N=float(abs(force[0])), force_on_q2_world=force_world))
        return contacts

    def check_contacts(self):
        details = self.contact_details()
        self.last_contact_force = sum(c['normal_force_N'] for c in details)
        if len(details) > 0:
            self.last_contact_time = float(self.world.time)
        return self.last_contact_force

    def relative_state(self):
        p1, p2 = [self.data.site_xpos[s].copy() for s in self.sites]
        states = [self.world.get_quad_state(i) for i in (1,2)]
        R1, R2 = self.world.rotation(1), self.world.rotation(2)
        vs = []
        for s, R, site in zip(states, (R1,R2), self.sites):
            v = np.array([s.vel.x,s.vel.y,s.vel.z])
            w = np.array([s.omega.x,s.omega.y,s.omega.z])
            vs.append(v + R @ np.cross(w,self.model.site_pos[site]))
        Rrel = R1.T @ R2
        qrel = np.zeros(4)
        mujoco.mju_mat2Quat(qrel, Rrel.ravel())
        w1, w2 = [np.array([s.omega.x,s.omega.y,s.omega.z]) for s in states]
        return dict(position=p2-p1, velocity=vs[1]-vs[0], quaternion=qrel,
                    angle_rad=float(np.arccos(np.clip((np.trace(Rrel)-1)/2,-1,1))),
                    angular_velocity_world=R2@w2-R1@w1)

    def is_contact_confirmed(self):
        details = self.contact_details()
        if len(details) > 0:
            self.last_contact_time = float(self.world.time)
        return len(details) > 0 or (self.is_captured and (float(self.world.time) - self.last_contact_time) < self.contact_hold_duration)

    def capture_eligible(self, tolerance_scale=1.0):
        r, p = self.relative_state(), self.params
        return (self.is_contact_confirmed() and
                np.linalg.norm(r['position']) <= p['position_tolerance_m'] * tolerance_scale and
                np.linalg.norm(r['velocity']) <= p['relative_linear_velocity_mps'] * tolerance_scale and
                r['angle_rad'] <= p['orientation_tolerance_rad'] * tolerance_scale and
                np.linalg.norm(r['angular_velocity_world']) <= p['relative_angular_velocity_radps'] * tolerance_scale)

    def initialize_connected(self):
        """Explicit preconnected initial-condition fixture; never a flight capture shortcut."""
        if self.world.time != 0.:
            raise ValueError('Preconnection is only allowed at t=0')
        return self._engage(initialization=True)

    def engage_weld(self, tolerance_scale=1.5):
        if self.latched:
            return True
        if not self.capture_eligible(tolerance_scale=tolerance_scale):
            return False
        return self._engage()

    def _engage(self,initialization=False):
        before = self.snapshot()
        R1, R2 = self.world.rotation(1), self.world.rotation(2)
        x1 = self.data.xpos[self.world.q1_body_id]
        x2 = self.data.xpos[self.world.q2_body_id]
        self.captured_separation = R1.T @ (x2-x1)
        self.captured_quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(self.captured_quaternion, (R1.T@R2).ravel())
        # Compiled weld layout: [anchor2(3), anchor1(3), relative_quat(4), torquescale].
        # relpose translation in MJCF is ANCHOR1, not body separation when anchor2 != 0.
        anchor2 = self.model.site_pos[self.sites[1]].copy()
        anchor1 = R1.T @ (x2 + R2@anchor2 - x1)
        self.model.eq_data[self.world.weld_eq_id, :3] = anchor2
        self.model.eq_data[self.world.weld_eq_id, 3:6] = anchor1
        self.model.eq_data[self.world.weld_eq_id, 6:10] = self.captured_quaternion
        self.world.set_weld_active(True)
        self.latched = True
        self.is_captured = False
        self.last_capture_force = 0.
        if self.filter_contact_on_lock:
            self.set_connector_collision_active(False)
        anchors = self.spring_anchors()
        self.spring_rest_length = np.linalg.norm(anchors[1]-anchors[0])+self.preload_deflection
        self.spring_armed = not self.zero_preload
        self.world.forward()
        self.events.append(dict(event='INITIAL_CONNECTED' if initialization else 'DOCK',time=self.world.time,pre=before,post=self.snapshot(),
                                spring_charging_work_J=self.spring_energy()))
        return True

    def disengage_weld(self):
        if not self.world.is_weld_active():
            return False
        before = self.snapshot()
        self.world.set_weld_active(False)
        self.latched = False
        self.disengaged = True
        self.set_connector_collision_active(True)
        self.world.forward()
        self.events.append(dict(event='UNDOCK',time=self.world.time,pre=before,post=self.snapshot(),
                                spring_energy_J=self.spring_energy()))
        return True

    def capture_force_generalized(self):
        result = np.zeros(self.model.nv)
        if not self.capture_enabled or self.latched or self.disengaged:
            self.last_capture_force = 0.
            return result

        # Capture mechanism is physically inactive until first physical contact occurs
        if not self.has_contacted:
            if len(self.contact_details()) > 0:
                self.has_contacted = True
            else:
                self.last_capture_force = 0.
                return result

        r = self.relative_state()
        dist = float(np.linalg.norm(r['position']))
        angle = float(r['angle_rad'])

        # Hysteresis on capture envelope
        h_factor = 1.25 if self.is_captured else 1.0
        if dist > self.capture_distance * h_factor or angle > self.capture_angle * h_factor:
            self.is_captured = False
            self.last_capture_force = 0.
            return result

        self.is_captured = True
        p1 = self.data.site_xpos[self.sites[0]].copy()
        p2 = self.data.site_xpos[self.sites[1]].copy()
        delta = p2 - p1
        d_sites = float(np.linalg.norm(delta))
        axis = delta / max(d_sites, 1e-9)

        # 1. Finite attractive force: pulling together along connector axis
        f_attract = min(self.max_capture_force, self.capture_stiffness * max(0.0, self.capture_distance - d_sites))

        # 2. Viscous axial damping to dissipate kinetic impact energy
        v_rel = r['velocity']
        v_axial = float(np.dot(v_rel, axis))
        if v_axial > 0:
            # Separating: damping adds to attraction to dissipate kinetic energy and arrest rebound
            f_pull = min(self.max_capture_force, f_attract + self.capture_damping * v_axial)
        else:
            # Closing: damping cushions arrival, smoothly zeroing at rest
            f_pull = max(0.0, f_attract + 0.5 * self.capture_damping * v_axial)

        force_on_q1 = f_pull * axis
        force_on_q2 = -force_on_q1 # Newton's third law: strictly equal and opposite

        # 3. Rotational damping to align relative attitude smoothly
        w_rel = r['angular_velocity_world']
        torque_on_q1 = np.clip(self.capture_ang_damping * w_rel, -0.2, 0.2)
        torque_on_q2 = -torque_on_q1

        self.last_capture_force = float(abs(f_pull))

        mujoco.mj_applyFT(self.model, self.data, force_on_q1, torque_on_q1, p1, self.world.q1_body_id, result)
        mujoco.mj_applyFT(self.model, self.data, force_on_q2, torque_on_q2, p2, self.world.q2_body_id, result)
        return result

    def spring_compression(self):
        if not self.spring_armed:
            return 0.
        anchors = self.spring_anchors()
        return max(0.,self.spring_rest_length-np.linalg.norm(anchors[1]-anchors[0]))

    def spring_anchors(self):
        # Attach 4 mm behind each contact face: a central spring has a finite axis
        # even when the contact sites coincide, and conserves angular momentum.
        return [self.data.site_xpos[s] + sign*.004*self.world.rotation(i)[:,1]
                for i,s,sign in zip((1,2),self.sites,(-1.,1.))]

    def spring_energy(self):
        return .5*self.spring_stiffness*self.spring_compression()**2

    def spring_force_generalized(self):
        result = np.zeros(self.model.nv)
        compression = self.spring_compression()
        self.preload_force = 0.
        if compression <= 0.:
            return result
        anchors = self.spring_anchors()
        delta = anchors[1]-anchors[0]
        axis = delta/max(np.linalg.norm(delta),1e-12)
        velocities = []
        for i,anchor in enumerate(anchors,1):
            s = self.world.get_quad_state(i)
            omega = self.world.rotation(i)@np.array([s.omega.x,s.omega.y,s.omega.z])
            pos = np.array([s.pos.x,s.pos.y,s.pos.z])
            velocities.append(np.array([s.vel.x,s.vel.y,s.vel.z])+np.cross(omega,anchor-pos))
        speed = np.dot(velocities[1]-velocities[0],axis)
        self.preload_force = max(0.,self.spring_stiffness*compression-self.damping*speed)
        # Finite spring storage, equal/opposite forces at the physical connector sites.
        for b,anchor,sign in zip(self.world.body_ids,anchors,(-1.,1.)):
            mujoco.mj_applyFT(self.model,self.data,sign*self.preload_force*axis,np.zeros(3),
                             anchor,b,result)
        return result

    def weld_generalized_force(self):
        force = np.zeros(self.data.nefc)
        rows = ((self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) &
                (self.data.efc_id == self.world.weld_eq_id))
        force[rows] = self.data.efc_force[rows]
        result = np.zeros(self.model.nv)
        mujoco.mj_mulJacTVec(self.model,self.data,result,force)
        return result

    def snapshot(self):
        q = self.world.system_quantities()
        return {**{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in q.items()},
                'qpos':self.data.qpos.tolist(),'qvel':self.data.qvel.tolist(),
                'spring_energy_J':self.spring_energy()}
