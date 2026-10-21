"""Contact-confirmed passive capture and finite-energy release, no magnetic force law."""
import mujoco
import numpy as np
from .pair_collision_filter import PairCollisionFilter


class LatchManager:
    def __init__(self, world, zero_preload=True, preload_force=None):
        self.world, self.model, self.data = world, world.model, world.data
        self.params = world.config['docking']['latch_criteria']
        spring = world.config['docking']['release']['elastic_preload']
        capture_cfg = world.config['docking'].get('capture_mechanism', {})
        self.capture_enabled = bool(capture_cfg.get('enabled', True))
        self.capture_distance = float(capture_cfg.get('capture_distance_m', 0.006))
        self.capture_angle = float(capture_cfg.get('capture_angle_rad', 0.15))
        self.max_capture_force = float(capture_cfg.get('max_resultant_force_N', capture_cfg.get('max_attraction_force_N', 6.0)))
        self.capture_stiffness = float(capture_cfg.get('stiffness_N_per_m', 140.0))
        self.capture_damping = float(capture_cfg.get('damping_Ns_per_m', 10.0))
        self.capture_ang_damping = float(capture_cfg.get('angular_damping_Nms_per_rad', 0.04))
        self.filter_contact_on_lock = bool(capture_cfg.get('filter_contact_on_lock', True))
        self.contact_hold_duration = float(capture_cfg.get('contact_hold_duration_s', 0.040))
        self.capture_anchor_inset = float(capture_cfg.get('anchor_inset_m', .004))
        if (not np.isfinite([self.capture_distance,self.capture_stiffness,self.max_capture_force,
                self.capture_anchor_inset,self.capture_damping,self.capture_ang_damping,
                self.contact_hold_duration,self.capture_angle]).all()
                or min(self.capture_distance, self.capture_stiffness, self.max_capture_force, self.capture_anchor_inset) <= 0
                or min(self.capture_damping, self.capture_ang_damping, self.contact_hold_duration) < 0
                or not 0 < self.capture_angle < .5
                or 2*self.capture_anchor_inset*np.cos(1.25*self.capture_angle/2) <= 1.25*self.capture_distance):
            raise ValueError('Invalid capture parameters or degenerate capture anchor geometry')
        self.zero_preload = zero_preload
        self.spring_stiffness = float(spring['spring_stiffness_N_per_m'])
        self.preload_deflection = float(spring['preload_deflection_m'])
        if preload_force is not None:
            self.preload_deflection = float(preload_force)/self.spring_stiffness
        self.damping = float(spring['damping_Ns_per_m'])
        self.q1_conn_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'q1_conn_funnel')
        self.q2_conn_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'q2_conn_probe')
        self.sites = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'q{i}_conn_site') for i in (1,2)]
        self.collision_filter = PairCollisionFilter(self.model, self.q1_conn_geom, self.q2_conn_geom)
        self.world.step_observers.append(self.update_capture_state)
        self.world.force_step_observers.append(self.record_capture_step)
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
        self.current_contact = False
        self.collision_index = 0
        self.attempt_id = 0
        self.attempt_open = False
        self.capture_energy_blocked = False
        self.capture_reservoir_initial_J = self.capture_storage_ceiling()
        self.capture_reservoir_energy_J = self.capture_reservoir_initial_J
        self.capture_returned_energy_J = 0.
        self.capture_transferred_energy_J = 0.
        self.capture_discarded_energy_J = 0.
        self.capture_dissipated_energy_J = 0.
        self.capture_diagnostics = self._zero_capture_diagnostics()
        self.last_capture_diagnostics = self.capture_diagnostics.copy()
        self.set_connector_collision_active(True)
        self.world.set_weld_active(False)

    def set_connector_collision_active(self, active: bool):
        self.collision_filter.set_active(active)

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

    def emit(self, name, before=None, **details):
        self.events.append(dict(event=name, time=self.world.time, attempt_id=self.attempt_id,
                                collision_index=self.collision_index,
                                pre=self.snapshot() if before is None else before,
                                post=self.snapshot(), **details))

    def update_capture_state(self):
        """One lifecycle at real physics timestamps, also used in standalone fixtures."""
        contact = bool(self.contact_details())
        if contact:
            self.last_contact_time = self.world.time
        contact_started = contact and not self.current_contact
        contact_before = self.snapshot() if contact_started else None
        self.current_contact = contact
        if not contact:
            self.capture_energy_blocked = False
        if contact_started:
            if not self.attempt_open and not self.latched and not self.disengaged:
                self.attempt_id += 1
                self.attempt_open = True
            self.collision_index += 1
            self.emit('CONTACT', contact_before, normal_force_N=self.check_contacts())
        if self.latched or self.disengaged or not self.capture_enabled:
            return
        r = self.relative_state()
        scale = 1.25 if self.is_captured else 1.
        inside = (np.linalg.norm(r['position']) <= scale*self.capture_distance and
                  r['angle_rad'] <= scale*self.capture_angle)
        if self.is_captured and not inside:
            self.fail_capture('envelope_exit')
        elif not self.is_captured and contact and inside and not self.capture_energy_blocked:
            if not self.attempt_open:
                self.attempt_id += 1
                self.attempt_open = True
            before = self.snapshot()
            self.is_captured = True
            self.has_contacted = True
            charging = self.capture_potential()
            if charging > self.capture_reservoir_energy_J + 1e-12:
                self.is_captured = False
                self.fail_capture('finite_reservoir_depleted')
                return
            self.capture_reservoir_energy_J = max(0.,self.capture_reservoir_energy_J-charging)
            self.capture_transferred_energy_J += charging
            self.emit('CAPTURE_START', before, internal_storage_transfer_J=charging,
                      reservoir_energy_J=self.capture_reservoir_energy_J)
        elif self.attempt_open and not contact and not self.is_captured:
            self.fail_capture('contact_without_engagement')

    def fail_capture(self, reason):
        before = self.snapshot()
        returned = self.capture_potential()
        self.capture_reservoir_energy_J += returned
        self.capture_returned_energy_J += returned
        self.is_captured = False
        self.has_contacted = False
        self.last_capture_force = 0.
        self.last_contact_time = -1.
        self.attempt_open = False
        self.capture_energy_blocked = reason == 'finite_reservoir_depleted'
        self.emit('CAPTURE_FAILED', before, reason=reason, internal_storage_return_J=returned)

    def abort(self):
        before = self.snapshot()
        if self.world.is_weld_active():
            self.disengage_weld()
        if self.attempt_open or self.is_captured:
            self.fail_capture('abort')
        self.disengaged = True
        self.set_connector_collision_active(True)
        self.world.forward()
        self.emit('ABORT', before)

    def is_contact_confirmed(self):
        self.update_capture_state()
        # Mechanical tension persists inside the envelope, but locking additionally
        # requires current/recent face contact. Old contact cannot re-engage after
        # envelope exit, nor confirm a lock indefinitely.
        return self.current_contact or (self.is_captured and
            self.world.time-self.last_contact_time <= self.contact_hold_duration + 1e-12)

    def contact_state(self):
        if self.latched:
            return 'LOCKED'
        if self.disengaged:
            return 'RELEASED'
        if self.current_contact:
            return 'CONTACT'
        if self.is_captured:
            return ('CONTACT_HOLD' if self.world.time-self.last_contact_time <= self.contact_hold_duration
                    else 'ENGAGED')
        return 'DETACHED' if self.attempt_id else 'NO_CONTACT'

    def is_lock_contact_confirmed(self):
        confirmed = self.is_contact_confirmed()
        # Face contact inside a broad kinematic tolerance is not proof that
        # the finite-stroke mechanism has actually engaged (e.g. edge contact).
        return confirmed and (not self.capture_enabled or self.is_captured)

    def capture_eligible(self, tolerance_scale=1.0):
        if self.disengaged:
            return False
        r, p = self.relative_state(), self.params
        return (self.is_lock_contact_confirmed() and
                np.linalg.norm(r['position']) <= p['position_tolerance_m'] * tolerance_scale and
                np.linalg.norm(r['velocity']) <= p['relative_linear_velocity_mps'] * tolerance_scale and
                r['angle_rad'] <= p['orientation_tolerance_rad'] * tolerance_scale and
                np.linalg.norm(r['angular_velocity_world']) <= p['relative_angular_velocity_radps'] * tolerance_scale)

    def initialize_connected(self):
        """Explicit preconnected initial-condition fixture; never a flight capture shortcut."""
        if self.world.time != 0.:
            raise ValueError('Preconnection is only allowed at t=0')
        return self._engage(initialization=True)

    def engage_weld(self, tolerance_scale=1.0):
        if self.latched:
            return True
        if not self.capture_eligible(tolerance_scale=tolerance_scale):
            return False
        return self._engage()

    def _engage(self,initialization=False):
        before = self.snapshot()
        if initialization:
            self.disengaged = False
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
        self.capture_discarded_energy_J += self.capture_potential()
        self.latched = True
        self.is_captured = False
        self.attempt_open = False
        self.last_capture_force = 0.
        if self.filter_contact_on_lock:
            self.set_connector_collision_active(False)
        anchors = self.spring_anchors()
        self.spring_rest_length = np.linalg.norm(anchors[1]-anchors[0])+self.preload_deflection
        self.spring_armed = not self.zero_preload
        self.world.forward()
        self.emit('INITIAL_CONNECTED' if initialization else 'DOCK', before,
                  spring_charging_work_J=self.spring_energy())
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
        self.emit('UNDOCK', before, spring_energy_J=self.spring_energy())
        return True

    @staticmethod
    def _zero_capture_diagnostics():
        return dict(static_force_N=0., damping_force_N=0., resultant_force_N=0.,
                    axial_velocity_mps=0., damping_power_W=0., potential_J=0.,
                    force_on_q1_world=np.zeros(3), torque_on_q1_world=np.zeros(3))

    def capture_anchors(self):
        # Two tensile attachment points inset behind the faces; their line remains
        # finite even at coincident contact sites. Forces act AT these anchors.
        return [self.data.site_xpos[s] + sign*self.capture_anchor_inset*self.world.rotation(i)[:,1]
                for i,s,sign in zip((1,2),self.sites,(-1.,1.))]

    def capture_kinematics(self):
        anchors = self.capture_anchors()
        delta = anchors[1]-anchors[0]
        length = float(np.linalg.norm(delta))
        # The configured envelope guarantees a strictly positive length. Reject
        # invalid manual fixture mutations instead of inventing an arbitrary axis.
        if length < 1e-10:
            raise ValueError('Capture anchors coincide outside valid capture geometry')
        axis = delta/length
        velocities = []
        for i,point in enumerate(anchors,1):
            st = self.world.get_quad_state(i)
            w = self.world.rotation(i)@np.array([st.omega.x,st.omega.y,st.omega.z])
            v = np.array([st.vel.x,st.vel.y,st.vel.z])
            pos = np.array([st.pos.x,st.pos.y,st.pos.z])
            velocities.append(v+np.cross(w,point-pos))
        return anchors, axis, length, float(np.dot(velocities[1]-velocities[0],axis))

    def capture_storage_ceiling(self):
        """Initial finite precharge: upper bound of the tensile element potential.

        An ideal internal clutch transfers storage at engagement/rupture without
        changing qpos/qvel. This is an explicit SIMULATION assumption, not a
        measured magnetic field or a source of free per-contact charging work.
        """
        dc,k,cap=self.capture_distance,self.capture_stiffness,self.max_capture_force
        if k<=0.: return 0.
        plateau=max(0.,dc-cap/k)
        return min(cap,k*dc)*2*self.capture_anchor_inset + cap*plateau + .5*k*(dc-plateau)**2

    def capture_potential(self):
        """Nonnegative tensile storage, U(0)=0, dU/dL=F_static.

        Drawn from a finite reservoir at engagement, returned at rupture, and
        dissipated when locking. No new energy is supplied at a collision.
        """
        if not self.is_captured:
            return 0.
        if self.capture_stiffness <= 0.:
            return 0.
        anchors = self.capture_anchors()
        length = np.linalg.norm(anchors[1]-anchors[0])
        base = 2*self.capture_anchor_inset
        # Integral of min(Fmax,k*max(dc-max(L-base,0),0)) from L=0.
        dc,k,cap = self.capture_distance,self.capture_stiffness,self.max_capture_force
        plateau = max(0.,dc-cap/k)
        g = float(np.clip(length-base,0.,dc))
        u = min(cap,k*dc)*min(length,base)
        u += cap*min(g,plateau)
        if g > plateau:
            u += k*(dc*(g-plateau)-.5*(g*g-plateau*plateau))
        return float(u)

    def capture_force_generalized(self):
        result = np.zeros(self.model.nv)
        self.capture_diagnostics = self._zero_capture_diagnostics()
        self.last_capture_force = 0.
        if not self.capture_enabled or self.latched or self.disengaged or not self.is_captured:
            return result
        anchors,axis,length,speed = self.capture_kinematics()
        gap = max(0.,length-2*self.capture_anchor_inset)
        static = min(self.max_capture_force,self.capture_stiffness*max(0.,self.capture_distance-gap))
        # Unilateral tension: symmetric dashpot, clipped in resultant space.
        # Effective damping = resultant-static, hence -Fd*v <= 0 under clipping.
        pull = float(np.clip(static+self.capture_damping*speed,0.,self.max_capture_force))
        damping = pull-static
        wrel = self.relative_state()['angular_velocity_world']
        torque = np.clip(self.capture_ang_damping*wrel,-.2,.2)
        power = -damping*speed-float(torque@wrel)
        self.last_capture_force = pull
        self.capture_diagnostics = dict(static_force_N=static,damping_force_N=damping,
            resultant_force_N=pull,axial_velocity_mps=speed,damping_power_W=power,
            potential_J=self.capture_potential(),force_on_q1_world=pull*axis,
            torque_on_q1_world=torque.copy())
        for body,point,sign in zip(self.world.body_ids,anchors,(1.,-1.)):
            mujoco.mj_applyFT(self.model,self.data,sign*pull*axis,sign*torque,point,body,result)
        # Held over precisely the next world.step; reading telemetry never integrates.
        return result

    def record_capture_step(self):
        self.last_capture_diagnostics = self.capture_diagnostics.copy()
        self.capture_dissipated_energy_J -= self.capture_diagnostics['damping_power_W']*self.world.dt

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
                'spring_energy_J':self.spring_energy(), 'capture_potential_J':self.capture_potential(),
                'latched':self.latched, 'capture_engaged':self.is_captured,
                'current_contact':bool(self.contact_details()),
                'last_contact_time_s':self.last_contact_time,
                'capture_reservoir_energy_J':self.capture_reservoir_energy_J,
                'contact_hold':bool(self.is_captured and not getattr(self,'current_contact',False) and
                                    self.world.time-self.last_contact_time <= self.contact_hold_duration + 1e-12),
                'disengaged':self.disengaged, 'attempt_id':getattr(self,'attempt_id',0),
                'connector_pair_collision_active':self.collision_filter.active,
                'motor_actual_N':self.world.applied_thrusts.tolist(),
                'motor_target_N':self.world.target_thrusts.tolist(),
                'fsm_mode':str(self.mode_provider()).split('.')[-1] if hasattr(self,'mode_provider') else None}
