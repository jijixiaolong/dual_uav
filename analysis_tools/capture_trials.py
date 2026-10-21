"""Reproducible randomized capture design, independent of observed outcomes."""
import numpy as np

DEFAULT_SEEDS=(20261021,1042,7301)


def generate_trials(seed,count_per_class=6):
    rng=np.random.default_rng(seed)
    trials=[]
    for category in ('safe','boundary','reject'):
        for index in range(count_per_class):
            boundary=category=='boundary'
            attitude=.06 if boundary else .003
            velocity=.25 if boundary else .035
            omega=.6 if boundary else .04
            offset=rng.uniform(-.0003,.0003,3)
            offset[1]=rng.uniform(-.0004,.0004)
            if boundary:
                offset=rng.uniform(-.008,.008,3)
            if category=='reject':
                # Pad edge contact can occur, but persistent transverse offset is
                # outside both the 6 mm engagement and 20 mm lock tolerances.
                offset[0]=rng.uniform(.035,.039)
                velocity=.4
            relative_v=rng.uniform(-velocity,velocity,3)
            relative_v[1]=-abs(relative_v[1])
            rpy=rng.uniform(-attitude,attitude,3)
            angular=rng.uniform(-omega,omega,3)
            opts=dict(q2_initial_offset=offset.tolist(),
                q1_initial_state=dict(velocity_world_mps=(-.5*relative_v).tolist()),
                q2_initial_state=dict(velocity_world_mps=(.5*relative_v).tolist(),
                                      rpy_rad=rpy.tolist(),omega_body_radps=angular.tolist()),
                contact_stiffness_scale=float(rng.uniform(.8,1.2)),
                contact_damping_scale=float(rng.uniform(.8,1.2)),
                capture_parameters=dict(capture_stiffness=float(rng.uniform(112.,168.)),
                                        capture_damping=float(rng.uniform(8.,12.))),
                motor_tau_s=float(rng.uniform(.018,.032)),
                control_delay_s=float(rng.integers(0,4)*.002),
                mass_scales=rng.uniform(.97,1.03,2).tolist(),
                inertia_scales=rng.uniform(.96,1.04,(2,3)).tolist())
            trials.append(dict(id=f'{seed}_{category}_{index:02d}',seed=int(seed),category=category,
                expectation=('no_lock' if category=='reject' else 'capture_expected' if category=='safe' else 'outcome_unspecified'),
                scenario=dict(duration=1.8,dock_cmd_time=0.,undock_cmd_time=None,
                              q1_init_y=-.221,q2_init_y=.221,research_options=opts)))
    return trials


def trial_metrics(rows,events,criteria):
    docks=[e for e in events if e['event']=='DOCK']
    contacts=[e for e in events if e['event']=='CONTACT']
    dock=docks[0]['time'] if docks else None
    contact=contacts[0]['time'] if contacts else None
    first_collision_invalid=None
    if contacts:
        first_collision_invalid=not snapshot_eligible(contacts[0]['post'],criteria)
    false_locks=0
    continuous_violations=0
    for event in docks:
        window=[r for r in rows if event['time']-criteria['dwell_time_s']-1e-10 <= r['time'] <= event['time']+1e-10]
        violations=sum(not row_eligible(r,criteria) for r in window)
        continuous_violations += violations
        false_locks += int(not snapshot_eligible(event['pre'],criteria) or bool(violations))
    peak=lambda key: max((float(r[key]) for r in rows if r.get(key) is not None),default=None)
    interval=[r for r in rows if contact is not None and r['time']>=contact-1e-10 and (dock is None or r['time']<=dock+1e-10)]
    commands=np.array([[r[f'motor{k}_command_N'] for k in range(8)] for r in rows])
    ticks=[r for r in rows if r['control_updated']]
    tick_commands=np.array([[r[f'motor{k}_command_N'] for k in range(8)] for r in ticks])
    slope=(float(np.max(np.abs(np.diff(tick_commands,axis=0))/np.array([r['control_dt_s'] for r in ticks[1:]])[:,None]))
           if len(ticks)>1 else None)
    weld_force=max((np.linalg.norm([r[f'weld_q1_force_world_N_{a}'] for a in 'xyz']) for r in rows),default=0.)
    weld_torque=max((np.linalg.norm([r[f'weld_q1_couple_at_anchor_world_Nm_{a}'] for a in 'xyz']) for r in rows),default=0.)
    return dict(capture_success=bool(docks),false_lock_count=int(false_locks),
        first_collision_invalid=first_collision_invalid,continuous_dwell_violation_samples=continuous_violations,
        first_contact_time_s=contact,dock_time_s=dock,
        contact_to_lock_s=dock-contact if dock is not None and contact is not None else None,
        max_postcontact_distance_m=max((r['rel_dist'] for r in interval),default=None),
        peak_contact_normal_N=peak('contact_normal_force_N'),
        peak_integrated_step_contact_N=peak('step_contact_normal_N'),
        peak_static_capture_N=peak('capture_step_static_force_N'),
        peak_resultant_capture_N=peak('capture_step_resultant_force_N'),
        max_weld_anchor_error_m=peak('weld_anchor_error_m'),
        max_weld_attitude_error_rad=peak('weld_attitude_error_rad'),
        peak_internal_weld_force_N=float(weld_force),peak_internal_weld_couple_Nm=float(weld_torque),
        motor_saturation_fraction=float(np.mean((commands<=.05+1e-8)|(commands>=5.5-1e-8))) if commands.size else None,
        peak_motor_command_slew_Nps=slope,
        capture_failure_count=sum(e['event']=='CAPTURE_FAILED' for e in events),
        collision_count=len(contacts),attempt_count=max((e.get('attempt_id',0) for e in events),default=0),
        completed_steps=len(rows))


def snapshot_eligible(snapshot,criteria):
    """Independent lock audit from recorded rigid state, not LatchManager formulas."""
    from scipy.spatial.transform import Rotation
    q=np.asarray(snapshot['qpos']);v=np.asarray(snapshot['qvel'])
    r1=Rotation.from_quat(q[3:7][[1,2,3,0]]).as_matrix()
    r2=Rotation.from_quat(q[10:14][[1,2,3,0]]).as_matrix()
    a1=np.array([0,.22,0]);a2=-a1
    delta=q[7:10]+r2@a2-q[:3]-r1@a1
    speed=v[6:9]+r2@np.cross(v[9:12],a2)-v[:3]-r1@np.cross(v[3:6],a1)
    angle=Rotation.from_matrix(r1.T@r2).magnitude()
    wrel=r2@v[9:12]-r1@v[3:6]
    return bool(np.linalg.norm(delta)<=criteria['position_tolerance_m']+1e-10 and
        np.linalg.norm(speed)<=criteria['relative_linear_velocity_mps']+1e-10 and
        angle<=criteria['orientation_tolerance_rad']+1e-10 and
        np.linalg.norm(wrel)<=criteria['relative_angular_velocity_radps']+1e-10 and
        (snapshot.get('current_contact',False) or snapshot.get('contact_hold',False)))


def row_eligible(row,criteria):
    speed=np.linalg.norm([row['connector_relative_velocity_'+a] for a in 'xyz'])
    return bool(row['rel_dist']<=criteria['position_tolerance_m']+1e-10 and
        row['relative_attitude_rad']<=criteria['orientation_tolerance_rad']+1e-10 and
        speed<=criteria['relative_linear_velocity_mps']+1e-10 and
        row['relative_angular_speed_radps']<=criteria['relative_angular_velocity_radps']+1e-10 and
        (row['contact_actual'] or row['contact_held']))
