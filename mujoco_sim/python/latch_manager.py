"""Contact-confirmed passive capture and finite-energy release, no magnetic force law."""
import mujoco
import numpy as np


class LatchManager:
    def __init__(self, world, zero_preload=True, preload_force=None):
        self.world, self.model, self.data = world, world.model, world.data
        self.params = world.config['docking']['latch_criteria']
        spring = world.config['docking']['release']['elastic_preload']
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
        self.reset()

    def reset(self):
        self.latched = False
        self.last_contact_force = 0.
        self.spring_armed = False
        self.spring_rest_length = 0.
        self.preload_force = 0.
        self.events = []
        self.world.set_weld_active(False)

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
        self.last_contact_force = sum(c['normal_force_N'] for c in self.contact_details())
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

    def capture_eligible(self):
        r, p = self.relative_state(), self.params
        return (len(self.contact_details()) > 0 and
                np.linalg.norm(r['position']) <= p['position_tolerance_m'] and
                np.linalg.norm(r['velocity']) <= p['relative_linear_velocity_mps'] and
                r['angle_rad'] <= p['orientation_tolerance_rad'] and
                np.linalg.norm(r['angular_velocity_world']) <= p['relative_angular_velocity_radps'])

    def initialize_connected(self):
        """Explicit preconnected initial-condition fixture; never a flight capture shortcut."""
        if self.world.time != 0.:
            raise ValueError('Preconnection is only allowed at t=0')
        return self._engage(initialization=True)

    def engage_weld(self):
        if self.latched:
            return True
        if not self.capture_eligible():
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
        self.world.forward()
        self.events.append(dict(event='UNDOCK',time=self.world.time,pre=before,post=self.snapshot(),
                                spring_energy_J=self.spring_energy()))
        return True

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
