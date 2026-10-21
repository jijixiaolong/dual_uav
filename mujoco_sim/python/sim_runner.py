"""One execution path for headless validation, rendering and interactive viewing."""
import sys
from pathlib import Path
import mujoco
import numpy as np
from collections import deque

CORE_BUILD_DIR = Path(__file__).resolve().parents[2] / 'core' / 'build'
if str(CORE_BUILD_DIR) not in sys.path:
    sys.path.insert(0,str(CORE_BUILD_DIR))
import docking_core_py as dcp
from .mujoco_world import MuJoCoWorld
from .latch_manager import LatchManager
from .configuration import load_configuration
from .recorder import vec, record_state


def v3(x):
    return dcp.Vec3(*np.asarray(x,dtype=float))


def set_reference(sp, p, v=(0,0,0), a=(0,0,0)):
    sp.pos,sp.vel,sp.acc = v3(p),v3(v),v3(a)


def quintic(start_p,start_v,start_a,end_p,elapsed,duration):
    """C2 reference from measured switch state, timed from the confirmed event."""
    t = np.clip(elapsed,0.,duration)
    c0,c1,c2 = np.asarray(start_p),np.asarray(start_v),.5*np.asarray(start_a)
    D = duration
    matrix = np.array([[D**3,D**4,D**5],[3*D**2,4*D**3,5*D**4],[6*D,12*D**2,20*D**3]])
    rhs = np.array([np.asarray(end_p)-c0-c1*D-c2*D*D,-c1-2*c2*D,-2*c2])
    c3,c4,c5 = np.linalg.solve(matrix,rhs)
    p = c0+c1*t+c2*t**2+c3*t**3+c4*t**4+c5*t**5
    v = c1+2*c2*t+3*c3*t**2+4*c4*t**3+5*c5*t**4
    a = 2*c2+6*c3*t+12*c4*t**2+20*c5*t**3
    return p,v,a


class SimulationRunner:
    def __init__(self,xml_path=None,dt_sim=None,control_rate_hz=None,
                 compensator_type=dcp.CompensatorType.NONE,zero_preload=True):
        self.config = load_configuration()
        sim = self.config['experiments']['simulation']
        self.dt_sim = float(dt_sim or sim['timestep_s'])
        self.control_dt = 1./float(control_rate_hz or sim['control_rate_hz'])
        if self.control_dt < self.dt_sim:
            raise ValueError('Control rate must not exceed physics rate')
        if xml_path is None:
            xml_path = Path(__file__).resolve().parents[1]/'assets'/'dual_quads.xml'
        self.world = MuJoCoWorld(xml_path,dt=self.dt_sim)
        self.latch = LatchManager(self.world,zero_preload=zero_preload)
        self.q1_params,self.q2_params = [self._quad_params(i) for i in (1,2)]
        self.dock_params = dcp.DockingParams()
        dock = self.config['docking']
        self.dock_params.connector_offset_q1 = v3(dock['connector_offset_q1'])
        self.dock_params.connector_offset_q2 = v3(dock['connector_offset_q2'])
        for target,key in [('pos_tol','position_tolerance_m'),('rot_tol','orientation_tolerance_rad'),
                           ('lin_vel_tol','relative_linear_velocity_mps'),('ang_vel_tol','relative_angular_velocity_radps'),
                           ('dwell_time','dwell_time_s')]:
            setattr(self.dock_params,target,dock['latch_criteria'][key])
        self.gains,self.conn_gains = self._gains('single_quad'),self._gains('connected_quad')
        self.ctrl_q1 = dcp.GeometricSE3Controller(self.q1_params.mass,self.q1_params.inertia,self.gains)
        self.ctrl_q2 = dcp.GeometricSE3Controller(self.q2_params.mass,self.q2_params.inertia,self.gains)
        self.alloc_q1,self.alloc_q2 = [dcp.SingleQuadAllocator(p) for p in (self.q1_params,self.q2_params)]
        self.alloc_conn = dcp.ConnectedQuadAllocator(self.q1_params,self.q2_params,self.dock_params)
        self.ctrl_conn = dcp.GeometricSE3Controller(self.alloc_conn.total_mass(),self.alloc_conn.aggregate_inertia(),self.conn_gains)
        self.fsm = dcp.HybridFSM(self.dock_params)
        self.compensator = dcp.EventCompensator()
        self.configure_compensator(compensator_type)
        self.telemetry = []
        self.events = []

    def _quad_params(self,i):
        p,b = dcp.QuadParams(),getattr(self.world,f'q{i}_body_id')
        nominal = self.config['quadrotor']
        p.mass = float(self.world.model.body_mass[b])
        # Read the plant inertia in BODY axes, not unit inertia or inertial-frame diagonal.
        qi = self.world.model.body_iquat[b]
        Ri = np.zeros(9)
        mujoco.mju_quat2Mat(Ri,qi)
        Ri = Ri.reshape(3,3)
        inertia = Ri @ np.diag(self.world.model.body_inertia[b]) @ Ri.T
        p.inertia = dcp.Mat3(inertia.ravel().tolist())
        cfgI = nominal['inertia']
        expected_I = np.array([[cfgI['ixx'],cfgI['ixy'],cfgI['ixz']],
                               [cfgI['ixy'],cfgI['iyy'],cfgI['iyz']],
                               [cfgI['ixz'],cfgI['iyz'],cfgI['izz']]])
        if not np.isclose(p.mass,nominal['mass']) or not np.allclose(inertia,expected_I,atol=1e-10):
            raise ValueError('MJCF mass/inertia and quad_nominal.yaml differ; update both physical declarations')
        p.arm_length = nominal['arm_length']
        p.gravity = -self.world.model.opt.gravity[2]
        for key in ('min_thrust','max_thrust','torque_ratio','slew_rate_limit'):
            setattr(p,key,nominal['motor'][key])
        p.rotor_angles_deg = nominal['geometry']['rotor_angles_deg']
        p.rotor_directions = nominal['geometry']['rotor_directions']
        # Fail fast if geometry/actuator definitions drift from controller configuration.
        for k,angle in enumerate(p.rotor_angles_deg):
            site = mujoco.mj_name2id(self.world.model,mujoco.mjtObj.mjOBJ_SITE,f'q{i}_rotor{k}')
            expected = p.arm_length*np.array([np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))])
            if not np.allclose(self.world.model.site_pos[site,:2],expected,atol=1e-9):
                raise ValueError('Rotor positions differ between MJCF and configuration')
            actuator = 4*(i-1)+k
            if (not np.allclose(self.world.model.actuator_gear[actuator,:3],[0,0,1]) or
                    not np.allclose(self.world.model.actuator_gear[actuator,3:5],0) or
                    not np.isclose(self.world.model.actuator_gear[actuator,5],p.torque_ratio*p.rotor_directions[k]) or
                    not np.isclose(self.world.model.actuator_ctrlrange[actuator,1],p.max_thrust)):
                raise ValueError('MJCF rotor wrench or limits differ from controller configuration')
        conn = self.world.model.site(f'q{i}_conn_site').pos
        if not np.allclose(conn,self.config['docking'][f'connector_offset_q{i}']):
            raise ValueError('Connector reference differs between MJCF and docking configuration')
        return p

    def _gains(self,name):
        gains = dcp.SE3Gains()
        config = self.config['controllers'][name]
        for key in ('kp','kv','ki','i_limit'):
            setattr(gains,key,v3(config['position_gains'][key]))
        for key in ('kr','kw'):
            setattr(gains,key,v3(config['attitude_gains'][key]))
        gains.ki_rot = v3(config['attitude_gains']['ki'])
        gains.i_limit_rot = v3(config['attitude_gains']['i_limit'])
        return gains

    def configure_compensator(self,comp_type,coeff=None,A=None,G=None):
        self.compensator_type = comp_type
        learning = self.config['experiments']['learning']
        self.compensator.configure(comp_type,learning['decay_times'],learning['qp_box_limit_N'],learning['event_window_s'])
        if coeff is not None:
            self.compensator.set_fixed_coefficients(list(coeff))
        if A is not None and G is not None:
            self.compensator.set_eclr_matrices(A,G)

    def aggregate_state(self):
        quantities = self.world.system_quantities()
        s = dcp.RigidBodyState()
        s.pos,s.vel = v3(quantities['com']),v3(quantities['com_velocity'])
        # One persistent aggregate frame follows Q1; no quaternion sign/mean ambiguity.
        s.quat = self.world.get_quad_state(1).quat
        s.omega = v3(self.world.rotation(1).T@quantities['aggregate_omega_world'])
        s.timestamp = self.world.time
        return s

    def _capture_geometry(self):
        self.alloc_conn.update_geometry(v3(self.latch.captured_separation),dcp.Quat(*self.latch.captured_quaternion))
        self.ctrl_conn.update_inertia(self.alloc_conn.total_mass(),self.alloc_conn.aggregate_inertia())

    def _reset(self,q1_y,q2_y,preconnected):
        self.world.reset(q1_init_pos=(q1_y,1.),q2_init_pos=(q2_y,1.))
        self.latch.reset()
        self.fsm.reset()
        self.compensator.reset()
        for c in (self.ctrl_q1,self.ctrl_q2,self.ctrl_conn):
            c.reset()
        self.alloc_q1.reset()
        self.alloc_q2.reset()
        separation = np.array(self.config['docking']['nominal_separation_at_dock'])
        self.alloc_conn.update_geometry(v3(separation),dcp.Quat())
        self.alloc_conn.reset()
        self.sp_q1,self.sp_q2,self.sp_conn = dcp.Setpoint(),dcp.Setpoint(),dcp.Setpoint()
        self.telemetry,self.events = [],[]
        self.dock_event_time,self.release_event_time = None,None
        self.first_contact_time = None
        self.last_control_sample_time = None
        self.aggregate_control_active = bool(preconnected)
        self.research_release_checkpoint = None
        self.last_delayed_motor_command = self.world.applied_thrusts.copy()
        self.last_wrenches = {name:dcp.WrenchCommand() for name in ('q1','q2','aggregate')}
        if preconnected:
            self.latch.initialize_connected()
            self._capture_geometry()
            self.fsm.initialize_connected(0.)
            self.dock_event_time = 0.
        self.world.forward()

    def run_scenario(self,duration=7.,dock_cmd_time=1.2,undock_cmd_time=4.2,
                     approach_offset=0.,q1_init_y=-.6,q2_init_y=.6,preconnected=False,
                     connected_trajectory=False,step_callback=None,research_options=None,
                     controller_policy=None):
        """Log at physics rate. Control ticks quantize onto physics ticks (4/6ms at 500/200Hz)."""
        if preconnected:
            q1_init_y,q2_init_y = -.22,.22
        self._reset(q1_init_y,q2_init_y,preconnected)
        self.research_options = dict(research_options or {})
        options = self.research_options
        self.controller_policy = controller_policy
        if options.get('q2_initial_offset') or options.get('q2_yaw_rad'):
            adr = self.world.q2_qpos_adr
            self.world.data.qpos[adr:adr+3] += np.array(options.get('q2_initial_offset',[0,0,0]))
            yaw = float(options.get('q2_yaw_rad',0.))
            self.world.data.qpos[adr+3:adr+7] = [np.cos(yaw/2),0,0,np.sin(yaw/2)]
            self.world.forward()
        self.sp_q2.yaw = float(options.get('q2_yaw_rad',0.))
        self.world.motor_tau = float(options.get('motor_tau_s',self.config['quadrotor']['motor']['time_constant']))
        self.latch.preload_deflection = float(options.get('preload_deflection_m',
            self.config['docking']['release']['elastic_preload']['preload_deflection_m']))
        if controller_policy is not None:
            controller_policy.reset(self)
        start = [np.array([0.,q1_init_y,1.]),np.array([0.,q2_init_y,1.])]
        end = [np.array([0.,-.22,1.]),np.array([0.,.22+approach_offset,1.])]
        offset = np.array(options.get('q2_initial_offset',[0,0,0]))
        start[1] += offset
        end[1] += offset
        release_targets = [np.array([0.,-.6,1.]),np.array([0.,.6,1.])] if preconnected else start
        motor = self.world.applied_thrusts.copy()
        nominal,comp = motor.copy(),np.zeros(8)
        delay_steps = int(round(float(options.get('control_delay_s',0.))/self.dt_sim))
        if delay_steps < 0:
            raise ValueError('Control delay must be nonnegative')
        delayed_commands = deque([motor.copy() for _ in range(delay_steps)])
        frozen_motor = None
        next_control,last_control = 0.,None
        approach_commanded,release_commanded = preconnected,False
        conn_start,release_start = None,None
        if preconnected:
            s = self.aggregate_state()
            conn_start = (vec(s.pos),vec(s.vel),np.zeros(3))
        for _ in range(int(round(duration/self.dt_sim))):
            t = self.world.time
            if self.first_contact_time is None and self.latch.contact_details():
                self.first_contact_time = t
                state = self.latch.snapshot()
                self.latch.events.append(dict(event='CONTACT',time=t,pre=state,post=state,
                                              normal_force_N=self.latch.check_contacts()))
                if controller_policy is not None:
                    controller_policy.on_event(self,'CONTACT',t,motor.copy())
            control_updated = t+1e-10 >= next_control
            elapsed = self.control_dt if last_control is None else t-last_control
            if control_updated:
                next_control += self.control_dt
                last_control = t
                self.last_control_sample_time = t
                s1,s2 = self.world.get_quad_state(1),self.world.get_quad_state(2)
                if not approach_commanded and t >= dock_cmd_time:
                    self.fsm.command_approach()
                    approach_commanded = True
                connected_dwell = self.config['experiments']['simulation']['minimum_connected_hold_s']
                if (undock_cmd_time is not None and not release_commanded and t >= undock_cmd_time
                        and self.dock_event_time is not None and t-self.dock_event_time >= connected_dwell):
                    self.fsm.command_undock()
                    release_commanded = True
                previous = self.fsm.current_mode()
                self.fsm.set_capture_contact(self.latch.is_contact_confirmed())
                mode = self.fsm.update(s1,s2,t,elapsed)
                if previous != mode and mode == dcp.FlightMode.CAPTURE_PENDING:
                    state = self.latch.snapshot()
                    self.latch.events.append(dict(event='CAPTURE_READY',time=t,pre=state,post=state))
                if previous != mode and mode == dcp.FlightMode.CONNECTED:
                    if controller_policy is not None:
                        controller_policy.on_event(self,'DOCK',t,motor.copy())
                    if not self.latch.engage_weld():
                        raise RuntimeError('FSM requested capture without physical latch conditions')
                    self.dock_event_time = t
                    self._capture_geometry()
                    self.ctrl_conn.reset()
                    self.alloc_conn.set_previous_thrusts(motor.tolist())
                    s = self.aggregate_state()
                    conn_start = (vec(s.pos),vec(s.vel),.5*(vec(self.sp_q1.acc)+vec(self.sp_q2.acc)))
                    # Legacy learner is deliberately disabled during P0/P1 validation.
                    # Feature/axis inconsistencies are documented in stage1_experiment_audit.md.
                    if self.compensator_type != dcp.CompensatorType.NONE:
                        raise RuntimeError('Learning is gated pending stage 3-5 feature/metric repairs')
                if previous != mode and mode == dcp.FlightMode.SEPARATING:
                    integration = np.empty(mujoco.mj_stateSize(self.world.model,mujoco.mjtState.mjSTATE_INTEGRATION))
                    mujoco.mj_getState(self.world.model,self.world.data,integration,mujoco.mjtState.mjSTATE_INTEGRATION)
                    self.research_release_checkpoint = dict(time=t,integration_state=integration.tolist(),
                        eq_data=self.world.model.eq_data.tolist(),motor_actual=self.world.applied_thrusts.tolist(),
                        motor_target=self.world.target_thrusts.tolist(),motor_command=motor.tolist(),
                        pre=self.latch.snapshot())
                    if controller_policy is not None:
                        controller_policy.on_event(self,'UNDOCK',t,motor.copy())
                    if options.get('release_constraint',True):
                        self.latch.disengage_weld()
                    else:
                        state = self.latch.snapshot()
                        self.latch.events.append(dict(event='INTERVENTION',time=t,pre=state,post=state,
                                                       physical_weld_kept=True))
                    self.release_event_time = t
                    release_start = [(vec(s.pos),vec(s.vel),vec(self.sp_conn.acc)) for s in (s1,s2)]
                    if options.get('reset_integrators',True):
                        self.ctrl_q1.reset()
                        self.ctrl_q2.reset()
                    if options.get('allocator_state','carry') == 'hover_reset':
                        self.alloc_q1.reset()
                        self.alloc_q2.reset()
                    else:
                        self.alloc_q1.set_previous_thrusts(motor[:4].tolist())
                        self.alloc_q2.set_previous_thrusts(motor[4:].tolist())
                    if 'release_motor_tau_s' in options:
                        self.world.motor_tau = float(options['release_motor_tau_s'])
                    if options.get('freeze_motor_commands',False):
                        frozen_motor = motor.copy()
                self.aggregate_control_active = (self.world.is_weld_active() if self.release_event_time is None
                                                  else not options.get('switch_controller',True))
                if controller_policy is not None:
                    controller_policy.begin_control(self,t,elapsed)
                if mode == dcp.FlightMode.CAPTURE_PENDING or (mode == dcp.FlightMode.ALIGN and self.latch.is_captured):
                    for sp,target in zip((self.sp_q1,self.sp_q2),end):
                        set_reference(sp,target,np.zeros(3),np.zeros(3))
                elif mode in (dcp.FlightMode.APPROACH,dcp.FlightMode.ALIGN):
                    for sp,p,target in zip((self.sp_q1,self.sp_q2),start,end):
                        set_reference(sp,*quintic(p,np.zeros(3),np.zeros(3),target,t-dock_cmd_time,
                                                  options.get('approach_duration_s',self.config['experiments']['simulation']['approach_duration_s'])))
                elif self.aggregate_control_active:
                    if connected_trajectory:
                        # A gentle, exactly differentiated trajectory; smooth onset at capture.
                        age = max(0.,t-self.dock_event_time)
                        target = np.array([.12,.06,1.05])
                        set_reference(self.sp_conn,*quintic(*conn_start,target,age,2.))
                    else:
                        set_reference(self.sp_conn,*quintic(*conn_start,np.array([0.,0.,1.]),t-self.dock_event_time,
                                                           self.config['experiments']['simulation']['connected_reference_transition_s']))
                elif self.release_event_time is not None:
                    hold = options.get('release_reference','separate') == 'hold'
                    targets = [initial[0] for initial in release_start] if hold else release_targets
                    for sp,initial,target in zip((self.sp_q1,self.sp_q2),release_start,targets):
                        set_reference(sp,*quintic(*initial,target,t-self.release_event_time,
                                                  options.get('separation_duration_s',self.config['experiments']['simulation']['separation_duration_s'])))
                else:
                    for sp,p in zip((self.sp_q1,self.sp_q2),start):
                        set_reference(sp,p)
                if frozen_motor is not None:
                    nominal = frozen_motor.copy()
                elif self.aggregate_control_active:
                    w = self.ctrl_conn.compute(self.aggregate_state(),self.sp_conn,elapsed)
                    if controller_policy is not None:
                        w = controller_policy.correct_wrench(self,'aggregate',w,t,elapsed)
                    self.last_wrenches['aggregate'] = w
                    self.alloc_conn.set_previous_thrusts(motor.tolist())
                    nominal = np.asarray(self.alloc_conn.allocate(w,elapsed))
                else:
                    ws = [self.ctrl_q1.compute(s1,self.sp_q1,elapsed),self.ctrl_q2.compute(s2,self.sp_q2,elapsed)]
                    if controller_policy is not None:
                        ws = [controller_policy.correct_wrench(self,name,w,t,elapsed) for name,w in zip(('q1','q2'),ws)]
                    self.last_wrenches['q1'],self.last_wrenches['q2'] = ws
                    if not (self.release_event_time is not None and t == self.release_event_time and options.get('allocator_state')=='hover_reset'):
                        self.alloc_q1.set_previous_thrusts(motor[:4].tolist())
                        self.alloc_q2.set_previous_thrusts(motor[4:].tolist())
                    nominal = np.r_[self.alloc_q1.allocate(ws[0],elapsed),self.alloc_q2.allocate(ws[1],elapsed)]
                comp = (np.asarray(controller_policy.motor_delta(self,t),dtype=float)
                        if controller_policy is not None else np.zeros(8))
                # Common rotor-space actuator projection after all controller contributions.
                # Mode-specific wrenches (4 vs 8 motors) are never interpolated.
                lo = np.array([self.q1_params.min_thrust]*4+[self.q2_params.min_thrust]*4)
                hi = np.array([self.q1_params.max_thrust]*4+[self.q2_params.max_thrust]*4)
                rate = np.array([self.q1_params.slew_rate_limit]*4+[self.q2_params.slew_rate_limit]*4)
                motor += np.clip(np.clip(nominal+comp,lo,hi)-motor,-rate*elapsed,rate*elapsed)
            delayed_commands.append(motor.copy())
            self.last_delayed_motor_command = delayed_commands.popleft()
            self.world.step(self.last_delayed_motor_command)
            if controller_policy is not None:
                controller_policy.after_step(self)
            row = record_state(self,motor,nominal,comp,control_updated,elapsed)
            if controller_policy is not None:
                row.update(controller_policy.diagnostics(self))
            self.telemetry.append(row)
            if step_callback is not None and step_callback(self,row) is False:
                break
        self.events = self.latch.events.copy()
        return self.telemetry
